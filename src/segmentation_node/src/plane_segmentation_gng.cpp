#include "segmentation_node/plane_segmentation_gng.hpp"

#include <random>
#include <limits>
#include <mutex>
#include <algorithm>
#include <cmath>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>

#ifdef _OPENMP
  #include <omp.h>
#endif

GrowingNeuralGas::GrowingNeuralGas(const GNGParams& p) : params(p) {}

void GrowingNeuralGas::train(const std::vector<Eigen::Vector3f>& X, unsigned seed)
{
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

  for (int t=1; t<= params.train_steps; ++t) {
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
    nodes_[s1].w     += params.eps_b * (x - nodes_[s1].w);
    for (auto &kv : nodes_[s1].neigh_age) {
      int j = kv.first;
      nodes_[j].w += params.eps_n * (x - nodes_[j].w);
    }

    connect_(s1, s2);
    prune_old_edges_();

    if ((t % params.lambda_insert)==0 && (int)nodes_.size()<params.max_nodes) {
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

      nodes_[q].error *= params.alpha;
      nodes_[f].error *= params.alpha;
      nodes_[r_idx].error = nodes_[q].error;
    }

    for (auto &nd : nodes_) nd.error *= params.d;
  }
}

int GrowingNeuralGas::nearest(const Eigen::Vector3f& x) const 
{
  int s=-1; float dmin=std::numeric_limits<float>::max();
  for (int i=0;i<(int)nodes_.size();++i) 
  {
    float d = (x - nodes_[i].w).squaredNorm();
    if (d < dmin) { dmin=d; s=i; }
  }
  return s;
}

void GrowingNeuralGas::connect_(int i,int j) 
{
  nodes_[i].neigh_age[j]=0;
  nodes_[j].neigh_age[i]=0;
}

void GrowingNeuralGas::disconnect_(int i,int j) 
{
  nodes_[i].neigh_age.erase(j);
  nodes_[j].neigh_age.erase(i);
}

void GrowingNeuralGas::prune_old_edges_() 
{
  for (int i=0;i<(int)nodes_.size();++i) {
    auto it = nodes_[i].neigh_age.begin();
    while (it!=nodes_[i].neigh_age.end()){
      int j = it->first; int age = it->second;
      if (age > params.max_age) {
        nodes_[j].neigh_age.erase(i);
        it = nodes_[i].neigh_age.erase(it);
      } else ++it;
    }
  }
}

PlaneSegmentationGNG::PlaneSegmentationGNG(const std::string& name) : Node(name)
{
  // I/O
  declare_parameter<std::string>("input_topic",  "/circle_cloud");
  declare_parameter<std::string>("output_topic", "/segment");

  // Preprocess
  declare_parameter<double>("z_min", -5.0);
  declare_parameter<double>("z_max",  5.0);
  declare_parameter<double>("leaf_size", 0.05);
  declare_parameter<int>("k_normals", 90);

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
  declare_parameter<double>("planarity_ratio",  0.13);
  declare_parameter<double>("min_inlier_ratio", 0.80);

  // Temporal smoothing
  declare_parameter<int>("smooth_window", 10);
  declare_parameter<bool>("snap_to_plane", true);

  const auto in  = get_parameter("input_topic").as_string();
  const auto out = get_parameter("output_topic").as_string();

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    in, rclcpp::SensorDataQoS(),
    std::bind(&PlaneSegmentationGNG::cb, this, std::placeholders::_1));

  pub_       = create_publisher<sensor_msgs::msg::PointCloud2>(out, 10);
  pub_debug_ = create_publisher<sensor_msgs::msg::PointCloud2>(out + "_debug", 10);

  // cache param
  smooth_window_ = std::max<int>(1, static_cast<int>(get_parameter("smooth_window").as_int()));
  snap_to_plane_ = get_parameter("snap_to_plane").as_bool();

  RCLCPP_INFO(get_logger(), "GNG plane seg (parallel + smoothing): %s -> %s (win=%d, snap=%d)",
              in.c_str(), out.c_str(), smooth_window_, snap_to_plane_ ? 1 : 0);
}

void PlaneSegmentationGNG::push_and_smooth_(const Eigen::Vector3f& c_new,
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
  else n_avg = n_new;
}

void PlaneSegmentationGNG::cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  if (cloud->empty()) return;

  const float zmin  = (float)get_parameter("z_min").as_double();
  const float zmax  = (float)get_parameter("z_max").as_double();
  const float leaf  = (float)get_parameter("leaf_size").as_double();
  const int   k     = get_parameter("k_normals").as_int();

  pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(zmin,zmax);
    pass.filter(*roi);
  }

  if (roi->empty()) return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(roi);
    vg.setLeafSize(leaf,leaf,leaf);
    vg.filter(*down);
  }

  if (down->empty()) return;

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
    for (const auto& n : normals->points) 
    {
      Eigen::Vector3f v(n.normal_x, n.normal_y, n.normal_z);
      float L = v.norm();
      if (std::isfinite(L) && L > 1e-6f) feats.emplace_back(v / L);
    }
  #endif
    if (feats.size() < 10) 
    {
      RCLCPP_WARN(get_logger(), "Too few normals for GNG (%zu).", feats.size());
      return;
    }

    GNGParams gp;
    gp.max_nodes     = get_parameter("gng_max_nodes").as_int();
    gp.train_steps   = get_parameter("gng_train_steps").as_int();
    gp.max_age       = get_parameter("gng_max_age").as_int();
    gp.lambda_insert = get_parameter("gng_lambda").as_int();
    gp.eps_b         = (float)get_parameter("gng_eps_b").as_double();
    gp.eps_n         = (float)get_parameter("gng_eps_n").as_double();
    gp.alpha         = (float)get_parameter("gng_alpha").as_double();
    gp.d             = (float)get_parameter("gng_d").as_double();

    GrowingNeuralGas gng(gp);
    gng.train(feats, /*seed*/1);
    const auto& nodes = gng.nodes();
    if (nodes.size()<2) 
    {
      RCLCPP_WARN(get_logger(), "GNG produced too few nodes.");
      return;
    }

    std::vector<int> cluster_idx(down->size(), -1);
  
  #ifdef _OPENMP
    #pragma omp parallel for
  #endif
    for (int i=0;i<(int)down->size();++i) {
      const auto& n = normals->points[i];
      Eigen::Vector3f v(n.normal_x, n.normal_y, n.normal_z);
      float L = v.norm();
      if (!(L>1e-6f) || !std::isfinite(L)) { cluster_idx[i]=-1; continue; }
      v/=L;

      int s=-1; float dmin=1e30f;
      for (int j=0;j<(int)nodes.size();++j) {
        float d = (v - nodes[j].w).squaredNorm();
        if (d < dmin) { dmin=d; s=j; }
      }
      cluster_idx[i] = s;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters(nodes.size());
    for (auto& c : clusters) c.reset(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::mutex> cluster_mtx(nodes.size());

  #ifdef _OPENMP
    #pragma omp parallel for
  #endif
    for (int i=0;i<(int)down->size();++i) 
    {
      int cid = cluster_idx[i];
      if (cid>=0) {
        std::lock_guard<std::mutex> lock(cluster_mtx[cid]);
        clusters[cid]->push_back(down->points[i]);
      }
    }

    const float dist_thr        = (float)get_parameter("plane_dist_thresh").as_double();
    const float planarity_ratio = (float)get_parameter("planarity_ratio").as_double();
    const float min_inlier_ratio= (float)get_parameter("min_inlier_ratio").as_double();

    auto color_for = [](int i)->Eigen::Vector3i 
    {
      static const int C[12][3] = {
        {230,25,75},{60,180,75},{255,225,25},{0,130,200},
        {245,130,48},{145,30,180},{70,240,240},{240,50,230},
        {210,245,60},{250,190,190},{0,128,128},{230,190,255}
      };
      auto c = C[i%12]; return Eigen::Vector3i(c[0],c[1],c[2]);
    };

    std::vector<pcl::PointXYZRGB> dbg_all;
    std::vector<pcl::PointXYZ>    plane_all;

    Eigen::Vector3f c_weighted_sum = Eigen::Vector3f::Zero();
    Eigen::Vector3f n_weighted_sum = Eigen::Vector3f::Zero();
    int             total_weight   = 0;

  #ifdef _OPENMP
    {
      int T = omp_get_max_threads();
      std::vector<std::vector<pcl::PointXYZRGB>> dbg_local(T);
      std::vector<std::vector<pcl::PointXYZ>>    plane_local(T);
      std::vector<Eigen::Vector3f> c_accum(T, Eigen::Vector3f::Zero());
      std::vector<Eigen::Vector3f> n_accum(T, Eigen::Vector3f::Zero());
      std::vector<int>             w_accum(T, 0);

      #pragma omp parallel
      {
        int tid = omp_get_thread_num();
        dbg_local[tid].reserve(4096);
        plane_local[tid].reserve(4096);

        #pragma omp for nowait
        for (int cid=0; cid<(int)clusters.size(); ++cid) 
        {
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
          for (const auto& p : pc->points) 
          {
            Eigen::Vector3f v(p.x,p.y,p.z);
            float dist = std::abs( (v-c).dot(n) );
            if (dist <= dist_thr) inliers++;
          }

          float ratio = (float)inliers / (float)pc->size();
          if (ratio < min_inlier_ratio) continue;

          c_accum[tid] += c * (float)inliers;
          n_accum[tid] += n * (float)inliers;
          w_accum[tid] += inliers;

          const auto col = color_for(cid);
          for (const auto& p : pc->points) 
          {
            pcl::PointXYZRGB q; q.x=p.x; q.y=p.y; q.z=p.z;
            q.r=col[0]; q.g=col[1]; q.b=col[2];
            dbg_local[tid].push_back(q);
          }
          
          for (const auto& p : pc->points) 
          {
            Eigen::Vector3f v(p.x,p.y,p.z);
            float dist = std::abs( (v-c).dot(n) );
            if (dist <= dist_thr) plane_local[tid].push_back(p);
          }
        }
      }

      size_t dbg_total=0, plane_total=0;
      for (auto& v: dbg_local)   dbg_total += v.size();
      for (auto& v: plane_local) plane_total += v.size();
      dbg_all.reserve(dbg_total);
      plane_all.reserve(plane_total);
      for (auto& v: dbg_local)   dbg_all.insert(dbg_all.end(), v.begin(), v.end());
      for (auto& v: plane_local) plane_all.insert(plane_all.end(), v.begin(), v.end());

      for (int t=0;t<T;++t) {
        c_weighted_sum += c_accum[t];
        n_weighted_sum += n_accum[t];
        total_weight   += w_accum[t];
      }
    }
  #else
    for (size_t cid=0; cid<clusters.size(); ++cid) 
    {
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
      for (const auto& p : pc->points) 
      {
        Eigen::Vector3f v(p.x,p.y,p.z);
        float dist = std::abs( (v-c).dot(n) );
        if (dist <= dist_thr) inliers++;
      }
      
      float ratio = (float)inliers / (float)pc->size();
      if (ratio < min_inlier_ratio) continue;

      c_weighted_sum += c * (float)inliers;
      n_weighted_sum += n * (float)inliers;
      total_weight   += inliers;

      const auto col = color_for((int)cid);
      for (const auto& p : pc->points) 
      {
        pcl::PointXYZRGB q; q.x=p.x; q.y=p.y; q.z=p.z;
        q.r=col[0]; q.g=col[1]; q.b=col[2];
        dbg_all.push_back(q);
      }
      
      for (const auto& p : pc->points) 
      {
        Eigen::Vector3f v(p.x,p.y,p.z);
        float dist = std::abs( (v-c).dot(n) );
        if (dist <= dist_thr) plane_all.push_back(p);
      }
    }
  #endif
    Eigen::Vector3f c_now = Eigen::Vector3f::Zero();
    Eigen::Vector3f n_now = Eigen::Vector3f::UnitZ();
    if (total_weight > 0) 
    {
      c_now = c_weighted_sum / (float)total_weight;
      n_now = n_weighted_sum;
      if (n_now.norm() > 1e-9f) n_now.normalize();
      else n_now = Eigen::Vector3f::UnitZ();
    }

    Eigen::Vector3f c_avg, n_avg;
    if (total_weight > 0) 
    {
      push_and_smooth_(c_now, n_now, c_avg, n_avg);
    } else 
    {
      if (!centroid_buf_.empty()) 
      {
        c_avg = centroid_buf_.back();
        n_avg = normal_buf_.back();
        if (n_avg.norm() > 1e-9f) n_avg.normalize();
      } else 
      {
        c_avg = c_now;
        n_avg = n_now;
      }
    }

    pcl::PointCloud<pcl::PointXYZ> plane_only;
    plane_only.reserve(plane_all.size());
    if (snap_to_plane_) 
    {
      for (auto& p : plane_all) 
      {
        Eigen::Vector3f v(p.x,p.y,p.z);
        Eigen::Vector3f v_proj = v - ((v - c_avg).dot(n_avg)) * n_avg;
        pcl::PointXYZ q;
        q.x = v_proj.x(); q.y = v_proj.y(); q.z = v_proj.z();
        plane_only.push_back(q);
      }
    } else 
    {
      for (auto& p : plane_all) plane_only.push_back(p);
    }

    pcl::PointCloud<pcl::PointXYZRGB> debug_col;
    debug_col.reserve(dbg_all.size());
    for (auto& q : dbg_all) debug_col.push_back(q);

    sensor_msgs::msg::PointCloud2 msg_plane, msg_dbg;
    pcl::toROSMsg(plane_only, msg_plane);
    msg_plane.header = msg->header;
    pub_->publish(msg_plane);

    if (pub_debug_->get_subscription_count()>0) 
    {
      pcl::toROSMsg(debug_col, msg_dbg);
      msg_dbg.header = msg->header;
      pub_debug_->publish(msg_dbg);
    }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaneSegmentationGNG>("plane_segmentation_gng"));
  rclcpp::shutdown();
  return 0;
}
