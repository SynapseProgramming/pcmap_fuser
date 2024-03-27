// MIT License
//
// Copyright (c) 2024 Saurabh Gupta, Tiziano Guadagnino, Benedikt Mersch,
// Ignacio Vizzo, Cyrill Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <algorithm>
#include <pcmap_fuser/AlignRansac2D.hpp>
#include <random>
#include <utility>
#include <vector>
#include <iostream>


namespace map_closures {

Eigen::Isometry2d KabschUmeyamaAlignment2D(
    const std::vector<map_closures::PointPair> &keypoint_pairs) {


    // print out the coordinates of the keypoints
    for(auto &it : keypoint_pairs){
        std::cout<<"ref: "<<it.ref<<"\n";
        std::cout<<"query: "<<it.query<<"\n";
    } 


//  compute the centroids (mean value) of the keypoints
  auto mean = map_closures::PointPair();
  for (auto &it : keypoint_pairs) {
    mean.ref += it.ref; 
    mean.query += it.query;
  }
  mean.query /= keypoint_pairs.size();
  mean.ref /= keypoint_pairs.size();

  std::vector<Eigen::Matrix2d> transformed;
  transformed.resize(keypoint_pairs.size());

 // transform to the same origin
  for (int i = 0; i < transformed.size(); i++) {
    transformed[i] = (keypoint_pairs[i].ref - mean.ref) *
                     ((keypoint_pairs[i].query - mean.query).transpose());
  }
  auto covariance_matrix = Eigen::Matrix2d().setZero();
  for (int i = 0; i < transformed.size(); i++) {
    covariance_matrix += transformed[i];
  }

//   auto covariance_matrix = boost::compute::transform_reduce(
//       keypoint_pairs.cbegin(), keypoint_pairs.cend(),
//       Eigen::Matrix2d().setZero(),
//       [&](const auto &keypoint_pair) {
//         return (keypoint_pair.ref - mean.ref) *
//                ((keypoint_pair.query - mean.query).transpose());
//       },
//       std::plus<Eigen::Matrix2d>());



  Eigen::JacobiSVD<Eigen::Matrix2d> svd(
      covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Isometry2d T = Eigen::Isometry2d::Identity();

  //compute rotation matrix
  const Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
  T.linear() = R.determinant() > 0 ? R : -R;
  T.translation() = mean.query - R * mean.ref;

  return T;
}

static constexpr double inliers_distance_threshold = 3.0;

// RANSAC Parameters
static constexpr double inliers_ratio = 0.3;
static constexpr double probability_success = 0.999;
static constexpr int min_points = 2;
static constexpr int __RANSAC_TRIALS__ =
    std::ceil(std::log(1.0 - probability_success) /
              std::log(1.0 - std::pow(inliers_ratio, min_points)));


PointPair::PointPair(const Eigen::Vector2d &r, const Eigen::Vector2d &q)
    : ref(r), query(q) {}

std::pair<Eigen::Isometry2d, int> RansacAlignment2D(
    const std::vector<PointPair> &keypoint_pairs) {
  const size_t max_inliers = keypoint_pairs.size();

  std::vector<PointPair> sample_keypoint_pairs(2);
  std::vector<int> inlier_indices;
  inlier_indices.reserve(max_inliers);

  std::vector<int> optimal_inlier_indices;
  optimal_inlier_indices.reserve(max_inliers);

  int iter = 0;
  while (iter++ < __RANSAC_TRIALS__) {
    inlier_indices.clear();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(0, keypoint_pairs.size() - 1);
    for (int i = 0; i < 2; i++) {
      sample_keypoint_pairs[i] = keypoint_pairs[dist(gen)];
    }

    auto T = KabschUmeyamaAlignment2D(sample_keypoint_pairs);

    int index = 0;
    std::for_each(keypoint_pairs.cbegin(), keypoint_pairs.cend(),
                  [&](const auto &keypoint_pair) {
                    if ((T * keypoint_pair.ref - keypoint_pair.query).norm() <
                        inliers_distance_threshold)
                      inlier_indices.emplace_back(index);
                    index++;
                  });

    if (inlier_indices.size() > optimal_inlier_indices.size()) {
      optimal_inlier_indices = inlier_indices;
    }
  }

  const int num_inliers = optimal_inlier_indices.size();
  std::vector<PointPair> inlier_keypoint_pairs(num_inliers);
  std::transform(optimal_inlier_indices.cbegin(), optimal_inlier_indices.cend(),
                 inlier_keypoint_pairs.begin(),
                 [&](const auto index) { return keypoint_pairs[index]; });
  auto T = KabschUmeyamaAlignment2D(inlier_keypoint_pairs);
  return {T, num_inliers};
}
}  // namespace map_closures
