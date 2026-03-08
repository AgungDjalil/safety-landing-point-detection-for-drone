#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>
#include <random>
#include <unordered_map>
#include <limits>
#include <mutex>
#include <deque>              // NEW
#ifdef _OPENMP
  #include <omp.h>
#endif

// -------------------------------
// Minimal Growing Neural Gas (3D)
// -------------------------------
struct GNGNode {
  Eigen::Vector3f w;
  float error = 0.f;
  std::unordered_map<int,int> neigh_age;
};

struct GNGParams {
  int   max_nodes      = 40;
  int   max_age        = 50;
  int   lambda_insert  = 100;
  float eps_b          = 0.05f;
  float eps_n          = 0.005f;
  float alpha          = 0.5f;
  float d              = 0.995f;
  int   train_steps    = 5000;
};

class GrowingNeuralGas3D {
public:
  explicit GrowingNeuralGas3D(const GNGParams& p) : P(p) {}

  void train(const std::vector<Eigen::Vector3f>& X, unsigned seed=1) {
    if (X.size() < 2) return;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<size_t> pick(0, X.size()-1);

    nodes_.clear();
    // init 2 nodes
    GNGNode a, b;
    a.w = X[pick(rng)];
    b.w = X[pick(rng)];
    nodes_.push_back(a);
    nodes_.push_back(b);

    for (int t=1; t<=P.train_steps; ++t) {
      const Eigen::Vector3f& x = X[pick(rng)];

      // dua node terdekat
      int s1=-1, s2=-1; float d1=1e30f, d2=1e30f;
      for (int i=0;i<(int)nodes_.size();++i) {
        float d = (x - nodes_[i].w).squaredNorm();
        if (d < d1) { d2=d1; s2=s1; d1=d; s1=i; }
        else if (d < d2) { d2=d; s2=i; }
      }

      for (auto &kv : nodes_[s1].neigh_age) kv.second++;

      nodes_[s1].error += d1;
      nodes_[s1].w     += P.eps_b * (x - nodes_[s1].w);
      for (auto &kv : nodes_[s1].neigh_age) {
        int j = kv.first;
        nodes_[j].w += P.eps_n * (x - nodes_[j].w);
      }

      connect_(s1, s2);
      prune_old_edges_();

      if ( (t % P.lambda_insert)==0 && (int)nodes_.size()<P.max_nodes ) {
        int q=0; for (int i=1;i<(int)nodes_.size();++i)
          if (nodes_[i].error > nodes_[q].error) q=i;
        int f = -1; float maxerr = -1.f;
        for (auto &kv : nodes_[q].neigh_age) {
          int j = kv.first;
          if (nodes_[j].error > maxerr) { maxerr = nodes_[j].error; f=j; }
        }
        if (f<0) continue;

        GNGNode r;
        r.w = 0.5f*(nodes_[q].w + nodes_[f].w);
        r.error = (nodes_[q].error + nodes_[f].error)*0.5f;
        nodes_.push_back(r);
        int r_idx = (int)nodes_.size()-1;

        disconnect_(q,f);
        connect_(q,r_idx);
        connect_(r_idx,f);

        nodes_[q].error *= P.alpha;
        nodes_[f].error *= P.alpha;
        nodes_[r_idx].error = nodes_[q].error;
      }

      for (auto &nd : nodes_) nd.error *= P.d;
    }
  }

  int nearest(const Eigen::Vector3f& x) const {
    int s=-1; float dmin=1e30f;
    for (int i=0;i<(int)nodes_.size();++i) {
      float d = (x - nodes_[i].w).squaredNorm();
      if (d < dmin) { dmin=d; s=i; }
    }
    return s;
  }

  const std::vector<GNGNode>& nodes() const { return nodes_; }

private:
  GNGParams P;
  std::vector<GNGNode> nodes_;

  void connect_(int i,int j){
    nodes_[i].neigh_age[j]=0;
    nodes_[j].neigh_age[i]=0;
  }
  void disconnect_(int i,int j){
    nodes_[i].neigh_age.erase(j);
    nodes_[j].neigh_age.erase(i);
  }
  void prune_old_edges_(){
    for (int i=0;i<(int)nodes_.size();++i) {
      auto it = nodes_[i].neigh_age.begin();
      while (it!=nodes_[i].neigh_age.end()){
        int j = it->first; int age = it->second;
        if (age > P.max_age) {
          nodes_[j].neigh_age.erase(i);
          it = nodes_[i].neigh_age.erase(it);
        } else ++it;
      }
    }
  }
};

// ----------------------------------------------------------
// ROS2 Node: GNG-based plane segmentation (parallelized + smoothing)
// ----------------------------------------------------------
class PlaneSegmentationGNG : public rclcpp::Node {
public:
  PlaneSegmentationGNG() : Node("plane_segmentation_gng") {
    // I/O
    declare_parameter<std::string>("input_topic",  "/circle_cloud");
    declare_parameter<std::string>("output_topic", "/segment");
    // Preprocess
    declare_parameter<double>("z_min", -5.0);
    declare_parameter<double>("z_max",  5.0);
    declare_parameter<double>("leaf_size", 0.05);
    declare_parameter<int>("k_normals", 80);
    // GNG
    declare_parameter<int>("gng_max_nodes", 80);
    declare_parameter<int>("gng_train_steps", 1000);
    declare_parameter<int>("gng_max_age", 50);
    declare_parameter<int>("gng_lambda", 35);
    declare_parameter<double>("gng_eps_b", 0.04);
    declare_parameter<double>("gng_eps_n", 0.004);
    declare_parameter<double>("gng_alpha", 0.5);
    declare_parameter<double>("gng_d", 0.995);
    // Plane validation
    declare_parameter<double>("plane_dist_thresh", 0.13);
    declare_parameter<double>("planarity_ratio",  0.1);
    declare_parameter<double>("min_inlier_ratio", 0.80);
    // NEW: temporal smoothing params
    declare_parameter<int>("smooth_window", 7);
    declare_parameter<bool>("snap_to_plane", true);

    declare_parameter<bool>("viz_gng", true);
    viz_gng_ = get_parameter("viz_gng").as_bool();
    pub_gng_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/gng_markers", 10);

    const auto in  = get_parameter("input_topic").as_string();
    const auto out = get_parameter("output_topic").as_string();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      in, rclcpp::SensorDataQoS(),
      std::bind(&PlaneSegmentationGNG::cb, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out, 10);
    pub_debug_ = create_publisher<sensor_msgs::msg::PointCloud2>(out + "_debug", 10);

    // cache param
    smooth_window_ = std::max<int>(1, static_cast<int>(get_parameter("smooth_window").as_int()));
    snap_to_plane_ = get_parameter("snap_to_plane").as_bool();

    RCLCPP_INFO(get_logger(), "GNG plane seg (parallel + smoothing): %s -> %s (win=%d, snap=%d)",
                in.c_str(), out.c_str(), smooth_window_, snap_to_plane_ ? 1 : 0);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_gng_markers_;

  bool viz_gng_{true};
  // --- Temporal smoothing buffers (moving average)
  std::deque<Eigen::Vector3f> centroid_buf_;
  std::deque<Eigen::Vector3f> normal_buf_;
  int  smooth_window_{5};
  bool snap_to_plane_{true};

  void push_and_smooth_(const Eigen::Vector3f& c_new,
                        const Eigen::Vector3f& n_new,
                        Eigen::Vector3f& c_avg,
                        Eigen::Vector3f& n_avg)
  {
    centroid_buf_.push_back(c_new);
    normal_buf_.push_back(n_new);
    if ((int)centroid_buf_.size() > smooth_window_) centroid_buf_.pop_front();
    if ((int)normal_buf_.size()   > smooth_window_) normal_buf_.pop_front();

    c_avg.setZero();
    n_avg.setZero();
    for (auto& c : centroid_buf_) c_avg += c;
    for (auto& n : normal_buf_)   n_avg += n;
    c_avg /= static_cast<float>(centroid_buf_.size());
    if (n_avg.norm() > 1e-9f) n_avg.normalize();
    else n_avg = n_new; // fallback
  }

  void publish_gng_markers_(const std_msgs::msg::Header& hdr,
                          const std::vector<GNGNode>& nodes,
                          const std::vector<Eigen::Vector3f>& node_pos,
                          const std::vector<int>& node_cnt,
                          int min_points = 20)
  {
    if (!viz_gng_ || nodes.empty() || pub_gng_markers_->get_subscription_count()==0) return;

    visualization_msgs::msg::MarkerArray arr;

    // Bersihkan dulu agar tidak menumpuk
    {
      visualization_msgs::msg::Marker del;
      del.header = hdr;
      del.ns = "gng";
      del.id = 0;
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(del);
    }

    // Nodes sebagai bola pada centroid
    for (int i = 0; i < (int)nodes.size(); ++i) {
      if (i >= (int)node_pos.size() || i >= (int)node_cnt.size()) continue;
      if (node_cnt[i] < min_points) continue; // skip node yang sedikit anggotanya

      visualization_msgs::msg::Marker m;
      m.header = hdr;
      m.ns = "gng_nodes";
      m.id = i;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.pose.position.x = node_pos[i].x();
      m.pose.position.y = node_pos[i].y();
      m.pose.position.z = node_pos[i].z();
      m.scale.x = m.scale.y = m.scale.z = 0.07;
      m.color.a = 1.0;
      m.color.r = 0.1f; m.color.g = 0.9f; m.color.b = 0.2f;
      arr.markers.push_back(m);

      // (opsional) panah normal dari centroid (arah = nodes[i].w)
      visualization_msgs::msg::Marker arrow;
      arrow.header = hdr; arrow.ns = "gng_normals"; arrow.id = 100000 + i;
      arrow.type = visualization_msgs::msg::Marker::ARROW; arrow.action = arrow.ADD;
      arrow.scale.x = 0.01; arrow.scale.y = 0.02; arrow.scale.z = 0.02;
      arrow.color.a = 0.9; arrow.color.r = 0.2; arrow.color.g = 0.6; arrow.color.b = 1.0;
      geometry_msgs::msg::Point p,q;
      p.x = node_pos[i].x(); p.y = node_pos[i].y(); p.z = node_pos[i].z();
      q.x = p.x + 0.2f * nodes[i].w.x();
      q.y = p.y + 0.2f * nodes[i].w.y();
      q.z = p.z + 0.2f * nodes[i].w.z();
      arrow.points = {p,q};
      arr.markers.push_back(arrow);
    }

    // Edges: hubungkan centroid antar node bertetangga
    visualization_msgs::msg::Marker lines;
    lines.header = hdr;
    lines.ns = "gng_edges";
    lines.id = 0;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.scale.x = 0.01;
    lines.color.a = 1.0;
    lines.color.r = 1.0f; lines.color.g = 0.8f; lines.color.b = 0.2f;

    for (int i = 0; i < (int)nodes.size(); ++i) {
      if (i >= (int)node_pos.size() || i >= (int)node_cnt.size()) continue;
      if (node_cnt[i] < min_points) continue;
      for (const auto& kv : nodes[i].neigh_age) {
        int j = kv.first;
        if (j <= i) continue;
        if (j >= (int)node_pos.size() || j >= (int)node_cnt.size()) continue;
        if (node_cnt[j] < min_points) continue;

        geometry_msgs::msg::Point p1, p2;
        p1.x = node_pos[i].x(); p1.y = node_pos[i].y(); p1.z = node_pos[i].z();
        p2.x = node_pos[j].x(); p2.y = node_pos[j].y(); p2.z = node_pos[j].z();
        lines.points.push_back(p1);
        lines.points.push_back(p2);
      }
    }
    arr.markers.push_back(lines);

    pub_gng_markers_->publish(arr);
  }

  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // --- input ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    const float zmin  = (float)get_parameter("z_min").as_double();
    const float zmax  = (float)get_parameter("z_max").as_double();
    const float leaf  = (float)get_parameter("leaf_size").as_double();
    const int   k     = get_parameter("k_normals").as_int();

    // --- ROI in z ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
    {
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(zmin,zmax);
      pass.filter(*roi);
    }
    if (roi->empty()) return;

    // --- Voxel downsample ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
    {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(roi);
      vg.setLeafSize(leaf,leaf,leaf);
      vg.filter(*down);
    }
    if (down->empty()) return;

    // --- Normal estimation (OMP dari PCL) ---
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    {
      pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
      auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
      ne.setInputCloud(down);
      ne.setSearchMethod(tree);
      ne.setKSearch(std::max(6, k));
#ifdef _OPENMP
      ne.setNumberOfThreads(omp_get_max_threads());
#endif
      ne.compute(*normals);
    }

    // --- Fitur utk GNG: normal unit (nx,ny,nz) ---
    std::vector<Eigen::Vector3f> feats;
    feats.reserve(normals->size());
#ifdef _OPENMP
    {
      int T = omp_get_max_threads();
      std::vector<std::vector<Eigen::Vector3f>> local(T);
      #pragma omp parallel
      {
        int tid = omp_get_thread_num();
        local[tid].reserve(normals->size()/T + 32);
        #pragma omp for nowait
        for (int i=0; i<(int)normals->size(); ++i) {
          const auto& n = normals->points[i];
          Eigen::Vector3f v(n.normal_x, n.normal_y, n.normal_z);
          float L = v.norm();
          if (std::isfinite(L) && L > 1e-6f) local[tid].emplace_back(v / L);
        }
      }
      size_t total=0; for (auto& v: local) total += v.size();
      feats.reserve(total);
      for (auto& v: local) feats.insert(feats.end(), v.begin(), v.end());
    }
#else
    for (const auto& n : normals->points) {
      Eigen::Vector3f v(n.normal_x, n.normal_y, n.normal_z);
      float L = v.norm();
      if (std::isfinite(L) && L > 1e-6f) feats.emplace_back(v / L);
    }
#endif
    if (feats.size() < 10) {
      RCLCPP_WARN(get_logger(), "Too few normals for GNG (%zu).", feats.size());
      return;
    }

    // --- Train GNG (tetap sekuensial) ---
    GNGParams gp;
    gp.max_nodes     = get_parameter("gng_max_nodes").as_int();
    gp.train_steps   = get_parameter("gng_train_steps").as_int();
    gp.max_age       = get_parameter("gng_max_age").as_int();
    gp.lambda_insert = get_parameter("gng_lambda").as_int();
    gp.eps_b         = (float)get_parameter("gng_eps_b").as_double();
    gp.eps_n         = (float)get_parameter("gng_eps_n").as_double();
    gp.alpha         = (float)get_parameter("gng_alpha").as_double();
    gp.d             = (float)get_parameter("gng_d").as_double();

    GrowingNeuralGas3D gng(gp);
    gng.train(feats, /*seed*/1);
    const auto& nodes = gng.nodes();

    if (nodes.size()<2) {
      RCLCPP_WARN(get_logger(), "GNG produced too few nodes.");
      return;
    }

    // --- Assignment klaster per titik (PARALEL, thread-safe) ---
    std::vector<int> cluster_idx(down->size(), -1);
#ifdef _OPENMP
    #pragma omp parallel for
#endif
    for (int i=0;i<(int)down->size();++i) {
      const auto& n = normals->points[i];
      Eigen::Vector3f v(n.normal_x, n.normal_y, n.normal_z);
      float L = v.norm(); if (!(L>1e-6f) || !std::isfinite(L)) { cluster_idx[i]=-1; continue; }
      v/=L;

      int s=-1; float dmin=1e30f;
      for (int j=0;j<(int)nodes.size();++j) {
        float d = (v - nodes[j].w).squaredNorm();
        if (d < dmin) { dmin=d; s=j; }
      }
      cluster_idx[i] = s;
    }

    // --- Hitung centroid posisi (XYZ) setiap node GNG dari anggota klasternya ---
    std::vector<Eigen::Vector3f> node_pos(nodes.size(), Eigen::Vector3f::Zero());
    std::vector<int>             node_cnt(nodes.size(), 0);

    for (int i = 0; i < (int)down->size(); ++i) {
      int cid = cluster_idx[i];
      if (cid >= 0) {
        node_pos[cid] += Eigen::Vector3f(down->points[i].x,
                                        down->points[i].y,
                                        down->points[i].z);
        node_cnt[cid] += 1;
      }
    }
    for (size_t i = 0; i < node_pos.size(); ++i) {
      if (node_cnt[i] > 0) node_pos[i] /= (float)node_cnt[i];
    }

    // --- Refinement: sebarkan node di bidang menggunakan kombinasi normal + posisi ---
Eigen::Vector3f pmin( std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max());
Eigen::Vector3f pmax(-std::numeric_limits<float>::max(),
                     -std::numeric_limits<float>::max(),
                     -std::numeric_limits<float>::max());
for (const auto& p : down->points) {
  pmin.x() = std::min(pmin.x(), p.x); pmax.x() = std::max(pmax.x(), p.x);
  pmin.y() = std::min(pmin.y(), p.y); pmax.y() = std::max(pmax.y(), p.y);
  pmin.z() = std::min(pmin.z(), p.z); pmax.z() = std::max(pmax.z(), p.z);
}
const float extent = (pmax - pmin).norm();
const float pos_w  = 0.3f;   // bobot posisi
const float pos_s  = (extent > 1e-6f) ? (1.0f / extent) : 1.0f;
const float nor_w  = 1.0f - pos_w;

// Re-assign cluster dengan jarak gabungan normal + posisi
std::fill(node_cnt.begin(), node_cnt.end(), 0);
std::fill(node_pos.begin(), node_pos.end(), Eigen::Vector3f::Zero());

for (int i=0; i<(int)down->size(); ++i) {
  const auto& n = normals->points[i];
  Eigen::Vector3f v(n.normal_x, n.normal_y, n.normal_z);
  float L = v.norm(); if (!(L>1e-6f) || !std::isfinite(L)) { cluster_idx[i] = -1; continue; }
  v /= L;
  const Eigen::Vector3f p(down->points[i].x, down->points[i].y, down->points[i].z);

  int best = -1; float bestd = std::numeric_limits<float>::max();
  for (int j=0; j<(int)nodes.size(); ++j) {
    if (j >= (int)node_cnt.size()) continue;
    float dn = (v - nodes[j].w).squaredNorm();
    float dp = ((p - node_pos[j]) * pos_s).squaredNorm();
    float d  = nor_w * dn + pos_w * dp;
    if (d < bestd) { bestd = d; best = j; }
  }
  cluster_idx[i] = best;
  if (best >= 0) {
    node_pos[best] += p;
    node_cnt[best] += 1;
  }
}
for (size_t j=0; j<node_pos.size(); ++j)
  if (node_cnt[j] > 0) node_pos[j] /= (float)node_cnt[j];

// ✅ panggil marker setelah refinement
publish_gng_markers_(msg->header, nodes, node_pos, node_cnt, /*min_points=*/10);

    // --- Kumpulkan titik per klaster (PARALEL dgn mutex per klaster) ---
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters(nodes.size());
    for (auto& c : clusters) c.reset(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::mutex> cluster_mtx(nodes.size());

#ifdef _OPENMP
    #pragma omp parallel for
#endif
    for (int i=0;i<(int)down->size();++i) {
      int cid = cluster_idx[i];
      if (cid>=0) {
        std::lock_guard<std::mutex> lock(cluster_mtx[cid]);
        clusters[cid]->push_back(down->points[i]);
      }
    }

    // --- Validasi planarity + pewarnaan (PARALEL, buffer lokal → merge) ---
    const float dist_thr        = (float)get_parameter("plane_dist_thresh").as_double();
    const float planarity_ratio = (float)get_parameter("planarity_ratio").as_double();
    const float min_inlier_ratio= (float)get_parameter("min_inlier_ratio").as_double();

    auto color_for = [](int i)->Eigen::Vector3i {
      static const int C[12][3] = {
        {230,25,75},{60,180,75},{255,225,25},{0,130,200},
        {245,130,48},{145,30,180},{70,240,240},{240,50,230},
        {210,245,60},{250,190,190},{0,128,128},{230,190,255}
      };
      auto c = C[i%12]; return Eigen::Vector3i(c[0],c[1],c[2]);
    };

    std::vector<pcl::PointXYZRGB> dbg_all;
    std::vector<pcl::PointXYZ>    plane_all;

    // --- plus: kumpulkan info plane utk smoothing
    Eigen::Vector3f c_weighted_sum = Eigen::Vector3f::Zero();
    Eigen::Vector3f n_weighted_sum = Eigen::Vector3f::Zero();
    int             total_weight   = 0;

#ifdef _OPENMP
    {
      int T = omp_get_max_threads();
      std::vector<std::vector<pcl::PointXYZRGB>> dbg_local(T);
      std::vector<std::vector<pcl::PointXYZ>>    plane_local(T);
      // untuk weighted mean (pakai atomics sederhana)
      std::vector<Eigen::Vector3f> c_accum(T, Eigen::Vector3f::Zero());
      std::vector<Eigen::Vector3f> n_accum(T, Eigen::Vector3f::Zero());
      std::vector<int>             w_accum(T, 0);

      #pragma omp parallel
      {
        int tid = omp_get_thread_num();
        dbg_local[tid].reserve(4096);
        plane_local[tid].reserve(4096);

        #pragma omp for nowait
        for (int cid=0; cid<(int)clusters.size(); ++cid) {
          auto& pc = clusters[cid];
          if ((int)pc->size() < 200) continue;

          Eigen::Vector4f centroid;
          pcl::compute3DCentroid(*pc, centroid);
          Eigen::Matrix3f cov; pcl::computeCovarianceMatrixNormalized(*pc, centroid, cov);
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
          Eigen::Vector3f eval = es.eigenvalues();
          Eigen::Matrix3f evec = es.eigenvectors();
          int idx_min = 0; if (eval[1]<eval[idx_min]) idx_min=1; if (eval[2]<eval[idx_min]) idx_min=2;
          Eigen::Vector3f n = evec.col(idx_min).normalized();
          float curvature = eval[idx_min] / eval.sum();
          if (curvature > planarity_ratio) continue;

          const Eigen::Vector3f c = centroid.head<3>();
          int inliers=0;
          for (const auto& p : pc->points) {
            Eigen::Vector3f v(p.x,p.y,p.z);
            float dist = std::abs( (v-c).dot(n) );
            if (dist <= dist_thr) inliers++;
          }
          float ratio = (float)inliers / (float)pc->size();
          if (ratio < min_inlier_ratio) continue;

          // akumulasi untuk rata-rata tertimbang
          c_accum[tid] += c * (float)inliers;
          n_accum[tid] += n * (float)inliers;
          w_accum[tid] += inliers;

          const auto col = color_for(cid);
          for (const auto& p : pc->points) {
            pcl::PointXYZRGB q; q.x=p.x; q.y=p.y; q.z=p.z;
            q.r=col[0]; q.g=col[1]; q.b=col[2];
            dbg_local[tid].push_back(q);
          }
          for (const auto& p : pc->points) {
            Eigen::Vector3f v(p.x,p.y,p.z);
            float dist = std::abs( (v-c).dot(n) );
            if (dist <= dist_thr) plane_local[tid].push_back(p);
          }
        }
      } // parallel

      // merge cloud
      size_t dbg_total=0, plane_total=0;
      for (auto& v: dbg_local)   dbg_total += v.size();
      for (auto& v: plane_local) plane_total += v.size();
      dbg_all.reserve(dbg_total);
      plane_all.reserve(plane_total);
      for (auto& v: dbg_local)   dbg_all.insert(dbg_all.end(), v.begin(), v.end());
      for (auto& v: plane_local) plane_all.insert(plane_all.end(), v.begin(), v.end());

      // merge akumulasi rata-rata tertimbang
      for (int t=0;t<T;++t) {
        c_weighted_sum += c_accum[t];
        n_weighted_sum += n_accum[t];
        total_weight   += w_accum[t];
      }
    }
#else
    for (size_t cid=0; cid<clusters.size(); ++cid) {
      auto& pc = clusters[cid];
      if (pc->size() < 200) continue;

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*pc, centroid);
      Eigen::Matrix3f cov; pcl::computeCovarianceMatrixNormalized(*pc, centroid, cov);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
      Eigen::Vector3f eval = es.eigenvalues();
      Eigen::Matrix3f evec = es.eigenvectors();
      int idx_min = 0; if (eval[1]<eval[idx_min]) idx_min=1; if (eval[2]<eval[idx_min]) idx_min=2;
      Eigen::Vector3f n = evec.col(idx_min).normalized();
      float curvature = eval[idx_min] / eval.sum();
      if (curvature > planarity_ratio) continue;

      const Eigen::Vector3f c = centroid.head<3>();
      int inliers=0;
      for (const auto& p : pc->points) {
        Eigen::Vector3f v(p.x,p.y,p.z);
        float dist = std::abs( (v-c).dot(n) );
        if (dist <= dist_thr) inliers++;
      }
      float ratio = (float)inliers / (float)pc->size();
      if (ratio < min_inlier_ratio) continue;

      // akumulasi untuk rata-rata tertimbang
      c_weighted_sum += c * (float)inliers;
      n_weighted_sum += n * (float)inliers;
      total_weight   += inliers;

      const auto col = color_for((int)cid);
      for (const auto& p : pc->points) {
        pcl::PointXYZRGB q; q.x=p.x; q.y=p.y; q.z=p.z;
        q.r=col[0]; q.g=col[1]; q.b=col[2];
        dbg_all.push_back(q);
      }
      for (const auto& p : pc->points) {
        Eigen::Vector3f v(p.x,p.y,p.z);
        float dist = std::abs( (v-c).dot(n) );
        if (dist <= dist_thr) plane_all.push_back(p);
      }
    }
#endif

    // --- TEMPORAL SMOOTHING: moving average centroid & normal (berat ke inlier)
    Eigen::Vector3f c_now = Eigen::Vector3f::Zero();
    Eigen::Vector3f n_now = Eigen::Vector3f::UnitZ();
    if (total_weight > 0) {
      c_now = c_weighted_sum / (float)total_weight;
      n_now = n_weighted_sum;
      if (n_now.norm() > 1e-9f) n_now.normalize();
      else n_now = Eigen::Vector3f::UnitZ();
    } else {
      // tidak ada plane valid pada frame ini → jangan update buffer
      // tapi tetap publish apa adanya
    }

    Eigen::Vector3f c_avg, n_avg;
    if (total_weight > 0) {
      push_and_smooth_(c_now, n_now, c_avg, n_avg);
    } else {
      // gunakan nilai rata-rata terakhir (jika ada)
      if (!centroid_buf_.empty()) {
        c_avg = centroid_buf_.back();
        n_avg = normal_buf_.back();
        if (n_avg.norm() > 1e-9f) n_avg.normalize();
      } else {
        c_avg = c_now;
        n_avg = n_now;
      }
    }

    // --- SNAP/PROJECTION (opsional) ke bidang halus (c_avg, n_avg)
    pcl::PointCloud<pcl::PointXYZ> plane_only;
    plane_only.reserve(plane_all.size());
    if (snap_to_plane_) {
      for (auto& p : plane_all) {
        Eigen::Vector3f v(p.x,p.y,p.z);
        Eigen::Vector3f v_proj = v - ((v - c_avg).dot(n_avg)) * n_avg;
        pcl::PointXYZ q;
        q.x = v_proj.x(); q.y = v_proj.y(); q.z = v_proj.z();
        plane_only.push_back(q);
      }
    } else {
      for (auto& p : plane_all) plane_only.push_back(p);
    }

    // publish
    pcl::PointCloud<pcl::PointXYZRGB> debug_col;
    debug_col.reserve(dbg_all.size());
    for (auto& q : dbg_all) debug_col.push_back(q);

    sensor_msgs::msg::PointCloud2 msg_plane, msg_dbg;
    pcl::toROSMsg(plane_only, msg_plane);
    msg_plane.header = msg->header;
    pub_->publish(msg_plane);

    if (pub_debug_->get_subscription_count()>0) {
      pcl::toROSMsg(debug_col, msg_dbg);
      msg_dbg.header = msg->header;
      pub_debug_->publish(msg_dbg);
    }
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaneSegmentationGNG>());
  rclcpp::shutdown();
  return 0;
}
