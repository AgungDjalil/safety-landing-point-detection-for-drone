// include/segmentation_node/ddgng.hpp
#pragma once
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <cstddef>

struct Node {
  Eigen::Vector3f pos;
  float strength = 0.5f;
  float error    = 0.0f;
  float utility  = 1.0f;
  int   age      = 0;
  bool  is_flat  = false;
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  int   cov_count = 0;
};

struct Edge { int a, b; int age; };

class DDGNG {
public:
  DDGNG(std::size_t max_nodes, float lr, float nlr, int edge_max_age,
        float util_decay, float err_decay, float neighbor_radius, float flat_curv_thresh);
  ~DDGNG();

  DDGNG(const DDGNG&) = delete;
  DDGNG& operator=(const DDGNG&) = delete;
  DDGNG(DDGNG&&) noexcept = default;
  DDGNG& operator=(DDGNG&&) noexcept = default;

  void update(const std::vector<Eigen::Vector3f>& points,
              const std::vector<Eigen::Vector3f>& colors);

  const std::vector<Node>& nodes() const { return nodes_; }
  const std::vector<Edge>& edges() const { return edges_; }

  void reconstructColors(std::vector<Eigen::Vector3f>& out_colors) const;

private:
  // params
  std::size_t max_nodes_;
  float lr_, nlr_;
  int   edge_max_age_;
  float util_decay_, err_decay_;
  float neighbor_radius_;
  float flat_curv_thresh_;

  // state
  std::vector<Node> nodes_;
  std::vector<Edge> edges_;

  // helpers (CPU only)
  void initTwoSeeds(const std::vector<Eigen::Vector3f>& points);
  void cpuAssignAndAccumulate(const std::vector<Eigen::Vector3f>& points,
                              std::vector<int>& winner_idx,
                              std::vector<float>& winner_dist2) const;
  void adaptWinners(const std::vector<Eigen::Vector3f>& points,
                    const std::vector<int>& winner_idx);
  void ageAndPruneEdges();
  void maybeInsertNode();
  void updateCurvatureLabels();
};
