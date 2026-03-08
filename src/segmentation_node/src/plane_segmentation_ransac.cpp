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
#include <cstring>   // memcpy

class PlaneSegmentationRansac : public rclcpp::Node 
{
public:
    PlaneSegmentationRansac() : Node("plane_segmentation_ransac"), processing_(false)
    {
        // this->declare_parameter<std::string>("input_topic",   "/zed/zed_node/point_cloud/cloud_registered");
        this->declare_parameter<std::string>("input_topic",   "/dept_camera/points");
        this->declare_parameter<double>     ("z_min",         -5.0);
        this->declare_parameter<double>     ("z_max",          5.0);
        this->declare_parameter<double>     ("leaf_size",      0.07);   // [OPT-2] lebih besar → lebih sedikit poin
        this->declare_parameter<std::string>("plane_topic",   "/plane");
        this->declare_parameter<std::string>("outlier_topic", "/outlier");
        this->declare_parameter<std::string>("stats_topic",   "/segmentation_stats");
        this->declare_parameter<int>        ("max_iterations", 200);    // [OPT-3] turunkan dari 800
        this->declare_parameter<int>        ("k_search",       20);     // [OPT-4] turunkan dari 50
        this->declare_parameter<double>     ("normal_dist_weight", 0.03);
        this->declare_parameter<double>     ("distance_threshold",  0.05);

        // [OPT-5] Cache semua parameter sekali saja, bukan tiap callback
        cacheParameters();

        // [OPT-6] Watch parameter changes via callback
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

        RCLCPP_INFO(this->get_logger(), "PlaneSegmentationRansac initialized (realtime-optimized)");
    }

private:
    // ─── Cached parameters struct ───────────────────────────────────────────
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

    // ─── Reusable PCL objects (avoid re-allocation per frame) ───────────────
    // [OPT-7] Pre-allocate persistent PCL filter/segmentation objects
    pcl::PassThrough<pcl::PointXYZ>                              pass_filter_;
    pcl::VoxelGrid<pcl::PointXYZ>                                voxel_filter_;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>         ne_omp_;       // [OPT-1]
    pcl::search::KdTree<pcl::PointXYZ>::Ptr                      kd_tree_  { new pcl::search::KdTree<pcl::PointXYZ> };
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>  segmented_;
    pcl::ExtractIndices<pcl::PointXYZ>                           extractor_;

    // [OPT-8] Skip-frame flag: drop incoming frame if previous still processing
    std::atomic<bool> processing_;

    // ─── Valid-point counting: amortized over N frames ──────────────────────
    // [OPT-11] Raw-pointer scan is ~3× faster than PointCloud2ConstIterator.
    //          We run it every `valid_count_interval_` frames and cache the result.
    static constexpr int VALID_COUNT_INTERVAL = 5;  // hitung tiap 5 frame
    int    frame_counter_    {0};
    size_t cached_valid_pts_ {0};
    size_t cached_total_grid_{0};
    double cached_valid_pct_ {0.0};

    // Scan raw buffer untuk valid (finite) XYZ points menggunakan pointer aritmetik.
    // Jauh lebih cepat dari PointCloud2ConstIterator karena tidak ada overhead
    // field-lookup per langkah; offset dihitung sekali sebelum loop.
    static void countValidPointsRaw(
        const sensor_msgs::msg::PointCloud2 &cloud_msg,
        size_t &out_total,
        size_t &out_valid,
        double &out_percentage)
    {
        out_total = static_cast<size_t>(cloud_msg.width) * cloud_msg.height;
        out_valid = 0;

        // Cari offset field x, y, z dalam point_step
        uint32_t off_x = 0, off_y = 4, off_z = 8;  // default PointXYZ layout
        for (const auto &f : cloud_msg.fields) {
            if      (f.name == "x") off_x = f.offset;
            else if (f.name == "y") off_y = f.offset;
            else if (f.name == "z") off_z = f.offset;
        }

        const uint8_t *base     = cloud_msg.data.data();
        const uint32_t step     = cloud_msg.point_step;
        const size_t   n        = out_total;

        // Loop dengan raw pointer — compiler dapat auto-vectorize
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

    // ─── Helper: build colored RGB cloud in-place (no per-point branch) ─────
    // [OPT-9] Direct memory copy + bulk color fill instead of per-point struct push
    static sensor_msgs::msg::PointCloud2 colorizeCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
        uint8_t r, uint8_t g, uint8_t b,
        const std_msgs::msg::Header &header)
    {
        pcl::PointCloud<pcl::PointXYZRGB> colored;
        colored.points.resize(src->size());
        colored.width    = src->size();
        colored.height   = 1;
        colored.is_dense = true;

        const size_t n = src->size();
        for (size_t i = 0; i < n; ++i) {
            auto &p  = colored.points[i];
            const auto &sp = src->points[i];
            p.x = sp.x; p.y = sp.y; p.z = sp.z;
            p.r = r;    p.g = g;    p.b = b;
        }

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(colored, msg);
        msg.header = header;
        return msg;
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // [OPT-8] Drop frame if still busy
        if (processing_.exchange(true)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Dropping frame – previous still processing");
            return;
        }

        auto start_time = std::chrono::high_resolution_clock::now();

        // ── 1. Valid-point count (amortized: hanya tiap VALID_COUNT_INTERVAL frame) ──
        // [OPT-11] Raw pointer scan, jalankan hanya tiap N frame untuk hemat CPU
        ++frame_counter_;
        if (frame_counter_ >= VALID_COUNT_INTERVAL) {
            frame_counter_ = 0;
            countValidPointsRaw(*msg,
                cached_total_grid_,
                cached_valid_pts_,
                cached_valid_pct_);
        }
        // Pakai cached value untuk frame yang di-skip

        // ── 2. Convert ROS → PCL ─────────────────────────────────────────────
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) { processing_ = false; return; }

        const size_t original_size = cloud->size(); // total grid (termasuk NaN)

        // ── 3. PassThrough filter ────────────────────────────────────────────
        pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pass_filter_.setInputCloud(cloud);
        pass_filter_.setFilterFieldName("z");
        pass_filter_.setFilterLimits(params_.z_min, params_.z_max);
        pass_filter_.filter(*roi_cloud);

        // ── 3. Voxel downsampling ────────────────────────────────────────────
        pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        {
            const float ls = static_cast<float>(params_.leaf_size);
            voxel_filter_.setInputCloud(roi_cloud);
            voxel_filter_.setLeafSize(ls, ls, ls);
            voxel_filter_.filter(*ds_cloud);
        }

        if (ds_cloud->size() < 10) {
            RCLCPP_WARN(this->get_logger(), "Too few points after downsampling (%zu)", ds_cloud->size());
            processing_ = false;
            return;
        }

        // ── 4. Normal estimation (OMP parallel) ─────────────────────────────
        // [OPT-1] NormalEstimationOMP uses all CPU threads automatically
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne_omp_.setInputCloud(ds_cloud);
        ne_omp_.setSearchMethod(kd_tree_);
        ne_omp_.setKSearch(params_.k_search);   // [OPT-4] 20 instead of 50
        ne_omp_.compute(*normals);

        // ── 5. RANSAC plane segmentation ─────────────────────────────────────
        segmented_.setOptimizeCoefficients(true);
        segmented_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        segmented_.setMethodType(pcl::SAC_RANSAC);
        segmented_.setNormalDistanceWeight(static_cast<float>(params_.normal_dist_weight));
        segmented_.setDistanceThreshold(static_cast<float>(params_.distance_threshold));
        segmented_.setMaxIterations(params_.max_iterations);  // [OPT-3] 200 vs 800
        segmented_.setAxis(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
        segmented_.setEpsAngle(10.0f * static_cast<float>(M_PI) / 180.0f);
        segmented_.setInputCloud(ds_cloud);
        segmented_.setInputNormals(normals);

        pcl::ModelCoefficients coeffs;
        pcl::PointIndices inliers;
        segmented_.segment(inliers, coeffs);

        // ── 6. Extract plane / outlier ───────────────────────────────────────
        auto inliers_ptr = std::make_shared<pcl::PointIndices>(inliers);

        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_xyz  (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_xyz(new pcl::PointCloud<pcl::PointXYZ>);

        extractor_.setInputCloud(ds_cloud);
        extractor_.setIndices(inliers_ptr);
        extractor_.setNegative(false);
        extractor_.filter(*plane_xyz);

        extractor_.setNegative(true);
        extractor_.filter(*outlier_xyz);

        // ── 7. Colorize & publish ────────────────────────────────────────────
        pub_plane_  ->publish(colorizeCloud(plane_xyz,   255, 255, 255, msg->header));
        pub_outlier_->publish(colorizeCloud(outlier_xyz, 255,   0,   0, msg->header));

        // ── 8. Stats ─────────────────────────────────────────────────────────
        auto end_time = std::chrono::high_resolution_clock::now();
        double comp_ms = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time).count() / 1000.0;

        auto stats_msg = std_msgs::msg::String();
        stats_msg.data =
            "computation_time: "    + std::to_string(comp_ms)                   + " ms\n" +
            "total_grid: "          + std::to_string(cached_total_grid_)         + "\n"   +
            "valid_points: "        + std::to_string(cached_valid_pts_)          + "\n"   +
            "valid_percentage: "    + std::to_string(cached_valid_pct_)          + " %\n" +
            "data_bytes: "          + std::to_string(msg->data.size())           + "\n"   +
            "original_cloud_size: " + std::to_string(original_size)              + "\n"   +
            "roi_cloud_size: "      + std::to_string(roi_cloud->size())          + "\n"   +
            "downsampled_size: "    + std::to_string(ds_cloud->size())           + "\n"   +
            "plane_size: "          + std::to_string(plane_xyz->size())          + "\n"   +
            "outlier_size: "        + std::to_string(outlier_xyz->size());
        pub_stats_->publish(stats_msg);

        RCLCPP_INFO(this->get_logger(),
            "Comp: %.2f ms | Grid: %zu | Valid: %zu (%.1f%%) | ROI: %zu | DS: %zu | Plane: %zu | Outlier: %zu",
            comp_ms,
            cached_total_grid_, cached_valid_pts_, cached_valid_pct_,
            roi_cloud->size(), ds_cloud->size(),
            plane_xyz->size(), outlier_xyz->size());

        processing_ = false;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaneSegmentationRansac>());
    rclcpp::shutdown();
    return 0;
}