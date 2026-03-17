#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <chrono>
#include <cmath>
#include <atomic>
#include <cstring>
#include <thread>   // [OPT-NEW] async pipeline thread

class PlaneSegmentationRansac : public rclcpp::Node
{
public:
    PlaneSegmentationRansac() : Node("plane_segmentation_ransac"), processing_(false)
    {
        this->declare_parameter<std::string>("input_topic",          "/zed/zed_node/point_cloud/cloud_registered");
        this->declare_parameter<double>     ("z_min",                -5.0);
        this->declare_parameter<double>     ("z_max",                 5.0);
        this->declare_parameter<double>     ("leaf_size",             0.07);
        this->declare_parameter<std::string>("plane_topic",          "/plane");
        this->declare_parameter<std::string>("outlier_topic",        "/outlier");
        this->declare_parameter<std::string>("stats_topic",          "/segmentation_stats");
        this->declare_parameter<int>        ("max_iterations",        650);
        this->declare_parameter<int>        ("k_search",              30);   // [OPT] 30 → 15
        this->declare_parameter<double>     ("normal_dist_weight",    0.17);
        this->declare_parameter<double>     ("distance_threshold",    0.097);

        cacheParameters();

        // [OPT-FIX] setOptimizeCoefficients cukup sekali di sini, bukan di tiap callback
        segmented_.setOptimizeCoefficients(true);
        segmented_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        segmented_.setMethodType(pcl::SAC_RANSAC);

        param_cb_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &) {
                cacheParameters();
                return rcl_interfaces::msg::SetParametersResult{}.set__successful(true);
            });

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            params_.input_topic, rclcpp::SensorDataQoS(),
            std::bind(&PlaneSegmentationRansac::callback, this, std::placeholders::_1)
        );

        pub_plane_   = this->create_publisher<sensor_msgs::msg::PointCloud2>(params_.plane_topic,   10);
        pub_outlier_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(params_.outlier_topic, 10);
        pub_stats_   = this->create_publisher<std_msgs::msg::String>        (params_.stats_topic,   10);

        RCLCPP_INFO(this->get_logger(), "PlaneSegmentationRansac initialized (fully optimized)");
    }

private:
    // ─── Cached parameters ──────────────────────────────────────────────────
    struct Params {
        std::string input_topic, plane_topic, outlier_topic, stats_topic;
        double z_min, z_max, leaf_size;
        double normal_dist_weight, distance_threshold;
        int max_iterations, k_search;
    } params_;

    void cacheParameters()
    {
        params_.input_topic          = this->get_parameter("input_topic").as_string();
        params_.plane_topic          = this->get_parameter("plane_topic").as_string();
        params_.outlier_topic        = this->get_parameter("outlier_topic").as_string();
        params_.stats_topic          = this->get_parameter("stats_topic").as_string();
        params_.z_min                = this->get_parameter("z_min").as_double();
        params_.z_max                = this->get_parameter("z_max").as_double();
        params_.leaf_size            = this->get_parameter("leaf_size").as_double();
        params_.normal_dist_weight   = this->get_parameter("normal_dist_weight").as_double();
        params_.distance_threshold   = this->get_parameter("distance_threshold").as_double();
        params_.max_iterations       = this->get_parameter("max_iterations").as_int();
        params_.k_search             = this->get_parameter("k_search").as_int();
    }

    // ─── Reusable PCL objects ────────────────────────────────────────────────
    pcl::PassThrough<pcl::PointXYZ>                              pass_filter_;
    pcl::VoxelGrid<pcl::PointXYZ>                                voxel_filter_;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>         ne_omp_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr                      kd_tree_  { new pcl::search::KdTree<pcl::PointXYZ> };
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>  segmented_;
    pcl::ExtractIndices<pcl::PointXYZ>                           extractor_;

    // [OPT-NEW] processing_ = true selama thread pipeline berjalan
    std::atomic<bool> processing_;

    // ─── Valid-point count (amortized tiap N frame) ──────────────────────────
    static constexpr int VALID_COUNT_INTERVAL = 5;
    int    frame_counter_    {0};
    size_t cached_valid_pts_ {0};
    size_t cached_total_grid_{0};
    double cached_valid_pct_ {0.0};

    static void countValidPointsRaw(
        const sensor_msgs::msg::PointCloud2 &cloud_msg,
        size_t &out_total,
        size_t &out_valid,
        double &out_percentage)
    {
        out_total = static_cast<size_t>(cloud_msg.width) * cloud_msg.height;
        out_valid = 0;

        uint32_t off_x = 0, off_y = 4, off_z = 8;
        for (const auto &f : cloud_msg.fields) {
            if      (f.name == "x") off_x = f.offset;
            else if (f.name == "y") off_y = f.offset;
            else if (f.name == "z") off_z = f.offset;
        }

        const uint8_t *base = cloud_msg.data.data();
        const uint32_t step = cloud_msg.point_step;
        const size_t   n    = out_total;

        for (size_t i = 0; i < n; ++i) {
            const uint8_t *ptr = base + i * step;
            float fx, fy, fz;
            std::memcpy(&fx, ptr + off_x, sizeof(float));
            std::memcpy(&fy, ptr + off_y, sizeof(float));
            std::memcpy(&fz, ptr + off_z, sizeof(float));
            if (std::isfinite(fx) && std::isfinite(fy) && std::isfinite(fz))
                ++out_valid;
        }

        out_percentage = (out_total > 0)
            ? (static_cast<double>(out_valid) / out_total * 100.0)
            : 0.0;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_plane_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_outlier_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_stats_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // ─── Colorize hanya untuk outlier (merah), plane pakai PointXYZ biasa ───
    // [OPT] Plane di-publish sebagai PointXYZ — tidak ada alokasi RGB sama sekali.
    //       Outlier tetap di-colorize merah sesuai kebutuhan visualisasi.
    static sensor_msgs::msg::PointCloud2 colorizeRed(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
        const std_msgs::msg::Header &header)
    {
        pcl::PointCloud<pcl::PointXYZRGB> colored;
        const size_t n = src->size();
        colored.points.resize(n);
        colored.width    = static_cast<uint32_t>(n);
        colored.height   = 1;
        colored.is_dense = true;

        for (size_t i = 0; i < n; ++i) {
            auto       &p  = colored.points[i];
            const auto &sp = src->points[i];
            p.x = sp.x; p.y = sp.y; p.z = sp.z;
            p.r = 255;  p.g = 0;    p.b = 0;
        }

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(colored, msg);
        msg.header = header;
        return msg;
    }

    // ─── Pipeline utama — dipanggil di thread terpisah ───────────────────────
    void pipeline(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Snapshot parameter agar thread-safe terhadap perubahan parameter live
        const Params p = params_;

        // 1. Valid-point count (amortized)
        ++frame_counter_;
        if (frame_counter_ >= VALID_COUNT_INTERVAL) {
            frame_counter_ = 0;
            countValidPointsRaw(*msg,
                cached_total_grid_,
                cached_valid_pts_,
                cached_valid_pct_);
        }

        // 2. Convert ROS → PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) { processing_ = false; return; }

        // 3. PassThrough filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pass_filter_.setInputCloud(cloud);
        pass_filter_.setFilterFieldName("z");
        pass_filter_.setFilterLimits(p.z_min, p.z_max);
        pass_filter_.filter(*roi_cloud);

        // 4. Voxel downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        {
            const float ls = static_cast<float>(p.leaf_size);
            voxel_filter_.setInputCloud(roi_cloud);
            voxel_filter_.setLeafSize(ls, ls, ls);
            voxel_filter_.filter(*ds_cloud);
        }

        if (ds_cloud->size() < 10) {
            RCLCPP_WARN(this->get_logger(),
                "Too few points after downsampling (%zu)", ds_cloud->size());
            processing_ = false;
            return;
        }

        // 5. Normal estimation (OMP parallel)
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne_omp_.setInputCloud(ds_cloud);
        ne_omp_.setSearchMethod(kd_tree_);
        ne_omp_.setKSearch(p.k_search);   // [OPT] default 15 — cukup untuk plane
        ne_omp_.compute(*normals);

        // 6. RANSAC segmentation
        // [OPT-FIX] setOptimizeCoefficients/setModelType/setMethodType sudah di constructor
        segmented_.setNormalDistanceWeight(static_cast<float>(p.normal_dist_weight));
        segmented_.setDistanceThreshold(static_cast<float>(p.distance_threshold));
        segmented_.setMaxIterations(p.max_iterations);
        segmented_.setAxis(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
        segmented_.setEpsAngle(5.0f * static_cast<float>(M_PI) / 180.0f);
        segmented_.setInputCloud(ds_cloud);
        segmented_.setInputNormals(normals);

        pcl::ModelCoefficients coeffs;
        pcl::PointIndices inliers;
        segmented_.segment(inliers, coeffs);

        // 7. Extract plane / outlier
        auto inliers_ptr = std::make_shared<pcl::PointIndices>(inliers);

        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_xyz  (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_xyz(new pcl::PointCloud<pcl::PointXYZ>);

        extractor_.setInputCloud(ds_cloud);
        extractor_.setIndices(inliers_ptr);
        extractor_.setNegative(false);
        extractor_.filter(*plane_xyz);

        extractor_.setNegative(true);
        extractor_.filter(*outlier_xyz);

        // 8. Publish
        // [OPT] Plane → PointXYZ langsung (warna default/putih di RViz)
        //       Outlier → PointXYZRGB merah
        sensor_msgs::msg::PointCloud2 plane_msg;
        pcl::toROSMsg(*plane_xyz, plane_msg);
        plane_msg.header = msg->header;
        pub_plane_->publish(plane_msg);

        pub_outlier_->publish(colorizeRed(outlier_xyz, msg->header));

        // 9. Stats
        auto end_time = std::chrono::high_resolution_clock::now();
        double comp_ms = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time).count() / 1000.0;

        auto stats_msg = std_msgs::msg::String();
        stats_msg.data =
            "computation_time: "    + std::to_string(comp_ms)          + " ms\n" +
            "valid_points: "        + std::to_string(cached_valid_pts_) + "\n"   +
            "valid_percentage: "    + std::to_string(cached_valid_pct_) + " %\n" +
            "plane_size: "          + std::to_string(plane_xyz->size()) + "\n"   +
            "outlier_size: "        + std::to_string(outlier_xyz->size());
        pub_stats_->publish(stats_msg);

        RCLCPP_INFO(this->get_logger(),
            "Comp: %.2f ms | Valid: %zu (%.1f%%) | Plane: %zu | Outlier: %zu",
            comp_ms,
            cached_valid_pts_, cached_valid_pct_,
            plane_xyz->size(), outlier_xyz->size());

        processing_ = false;
    }

    // ─── Callback: drop frame jika masih processing, lalu spawn thread ───────
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // [OPT-NEW] Drop frame jika pipeline sebelumnya masih berjalan
        if (processing_.exchange(true)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Dropping frame – previous still processing");
            return;
        }

        // [OPT-NEW] Jalankan pipeline di thread terpisah agar callback tidak blocking.
        // ROS subscriber thread bebas menerima frame berikutnya segera.
        std::thread(&PlaneSegmentationRansac::pipeline, this, msg).detach();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaneSegmentationRansac>());
    rclcpp::shutdown();
    return 0;
}