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
#include <iostream>
#include <pcmap_fuser/AlignRansac2D.hpp>
#include <random>
#include <utility>
#include <vector>

namespace map_closures {

Eigen::Isometry2d KabschUmeyamaAlignment2D(
    const std::vector<map_closures::PointPair> &keypoint_pairs) {
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

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(
      covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Isometry2d T = Eigen::Isometry2d::Identity();

  // compute rotation matrix
  const Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
  T.linear() = R.determinant() > 0 ? R : -R;
  T.translation() = mean.query - R * mean.ref;

  return T;
}

PointPair::PointPair(const Eigen::Vector2d &r, const Eigen::Vector2d &q)
    : ref(r), query(q) {}

}  // namespace map_closures
