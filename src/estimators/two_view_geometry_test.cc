// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#define TEST_NAME "estimators/two_view_geometry"
#include "util/testing.h"

#include "base/camera_models.h"
#include "base/pose.h"
#include "estimators/two_view_geometry.h"

using namespace colmap;

BOOST_AUTO_TEST_CASE(TestDefault) {
  TwoViewGeometry two_view_geometry;
  BOOST_CHECK_EQUAL(two_view_geometry.config, TwoViewGeometry::UNDEFINED);
  BOOST_CHECK_EQUAL(two_view_geometry.F, Eigen::Matrix3d::Zero());
  BOOST_CHECK_EQUAL(two_view_geometry.E, Eigen::Matrix3d::Zero());
  BOOST_CHECK_EQUAL(two_view_geometry.H, Eigen::Matrix3d::Zero());
  BOOST_CHECK_EQUAL(two_view_geometry.qvec, Eigen::Vector4d::Zero());
  BOOST_CHECK_EQUAL(two_view_geometry.tvec, Eigen::Vector3d::Zero());
  BOOST_CHECK(two_view_geometry.inlier_matches.empty());
}

BOOST_AUTO_TEST_CASE(TestInvert) {
  TwoViewGeometry two_view_geometry;
  two_view_geometry.config = TwoViewGeometry::CALIBRATED;
  two_view_geometry.F = two_view_geometry.E = two_view_geometry.H =
      Eigen::Matrix3d::Identity();
  two_view_geometry.qvec = ComposeIdentityQuaternion();
  two_view_geometry.tvec = Eigen::Vector3d(0, 1, 2);
  two_view_geometry.inlier_matches.resize(2);
  two_view_geometry.inlier_matches[0] = FeatureMatch(0, 1);
  two_view_geometry.inlier_matches[1] = FeatureMatch(2, 3);

  two_view_geometry.Invert();
  BOOST_CHECK_EQUAL(two_view_geometry.config, TwoViewGeometry::CALIBRATED);
  BOOST_CHECK(two_view_geometry.F.isApprox(Eigen::Matrix3d::Identity()));
  BOOST_CHECK(two_view_geometry.E.isApprox(Eigen::Matrix3d::Identity()));
  BOOST_CHECK(two_view_geometry.H.isApprox(Eigen::Matrix3d::Identity()));
  BOOST_CHECK(two_view_geometry.qvec.isApprox(ComposeIdentityQuaternion()));
  BOOST_CHECK(two_view_geometry.tvec.isApprox(Eigen::Vector3d(-0, -1, -2)));
  BOOST_CHECK_EQUAL(two_view_geometry.inlier_matches[0].point2D_idx1, 1);
  BOOST_CHECK_EQUAL(two_view_geometry.inlier_matches[0].point2D_idx2, 0);
  BOOST_CHECK_EQUAL(two_view_geometry.inlier_matches[1].point2D_idx1, 3);
  BOOST_CHECK_EQUAL(two_view_geometry.inlier_matches[1].point2D_idx2, 2);

  two_view_geometry.Invert();
  BOOST_CHECK_EQUAL(two_view_geometry.config, TwoViewGeometry::CALIBRATED);
  BOOST_CHECK(two_view_geometry.F.isApprox(Eigen::Matrix3d::Identity()));
  BOOST_CHECK(two_view_geometry.E.isApprox(Eigen::Matrix3d::Identity()));
  BOOST_CHECK(two_view_geometry.H.isApprox(Eigen::Matrix3d::Identity()));
  BOOST_CHECK(two_view_geometry.qvec.isApprox(ComposeIdentityQuaternion()));
  BOOST_CHECK(two_view_geometry.tvec.isApprox(Eigen::Vector3d(0, 1, 2)));
  BOOST_CHECK_EQUAL(two_view_geometry.inlier_matches[0].point2D_idx1, 0);
  BOOST_CHECK_EQUAL(two_view_geometry.inlier_matches[0].point2D_idx2, 1);
  BOOST_CHECK_EQUAL(two_view_geometry.inlier_matches[1].point2D_idx1, 2);
  BOOST_CHECK_EQUAL(two_view_geometry.inlier_matches[1].point2D_idx2, 3);
}

namespace internal_test {
void CreateData(double f, size_t w, size_t h, Camera& cam1, Camera& cam2,
                std::vector<Eigen::Vector2d>& pts1,
                std::vector<Eigen::Vector2d>& pts2, FeatureMatches& matches,
                Eigen::Matrix3d& E, Eigen::Matrix3d& F, Eigen::Matrix3d& H,
                Eigen::Matrix3d& R, Eigen::Vector3d& t) {
  cam1.InitializeWithId(SimplePinholeCameraModel::model_id, f, w, h);
  cam2.InitializeWithId(SimplePinholeCameraModel::model_id, f, w, h);
  R = Eigen::AngleAxisd(-0.05 * M_PI, Eigen::Vector3d::UnitY());
  R = R.transpose();
  t = -R * Eigen::Vector3d(0.1, 0.0, 0.0);
  const Eigen::Matrix3d K =
      (Eigen::Matrix3d() << f, 0, w / 2, 0.0, f, h / 2.0, 0.0, 0.0, 1.0)
          .finished();
  const Eigen::Matrix3d K_inv = K.inverse();
  const Eigen::Matrix3d t_mat = (Eigen::Matrix3d() << 0.0, -t.z(), t.y(), t.z(),
                                 0.0, -t.x(), -t.y(), t.x(), 0.0)
                                    .finished();
  E = t_mat * R;
  E = E / E(2, 1);
  F = K_inv.transpose() * E * K_inv;
  F = F / F(2, 2);

  // create 3D points
  double depths[3] = {1, 1.2, 1.5};
  point2D_t idx = 0u;
  for (size_t i = 0; i < 5; i++) {
    double u1 = 100 + i * 30;
    for (size_t j = 0; j < 5; j++) {
      double v1 = 50 + j * 30;

      double d1 = depths[(i * 5 + j) % 3];
      // std::cout << d1 << std::endl;

      const Eigen::Vector3d ptw = K_inv * Eigen::Vector3d(u1, v1, 1) * d1;
      // std::cout << ptw.transpose() << std::endl;
      Eigen::Vector3d ptc = R * ptw + t;
      if (ptc.z() <= 0.0) {
        continue;
      }
      ptc /= ptc.z();
      const auto pti = K * ptc;
      if (pti.x() <= 0 || pti.x() >= w || pti.y() <= 0 || pti.y() >= h) {
        continue;
      }
      pts1.push_back(Eigen::Vector2d(u1, v1));
      pts2.push_back(Eigen::Vector2d(pti.x(), pti.y()));
      // std::cout << pts1.back() << "\n" << pts2.back() << std::endl;
      matches.push_back({idx, idx});
      ++idx;
    }
  }
  std::cout << "matches size = " << matches.size() << "\n";
}
}  // namespace internal_test

BOOST_AUTO_TEST_CASE(TestComputeRelativePose) {
  // Create dataset.
  Camera cam1, cam2;
  std::vector<Eigen::Vector2d> pts1, pts2;
  FeatureMatches matches;
  Eigen::Matrix3d E, F, H;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  size_t w = 320, h = 240;
  double f = 300;
  internal_test::CreateData(f, w, h, cam1, cam2, pts1, pts2, matches, E, F, H,
                            R, t);

  // Output GT.
  std::cout << "GT Pose:\n"
            << Eigen::Quaterniond(R.transpose()).coeffs() << "\n"
            << t << "\n";
  std::cout << "GT E: \n"
            << E << "\n"
            << "GT F: " << F << std::endl;

  // Call estimate.
  cam1.SetPriorFocalLength(false);
  cam2.SetPriorFocalLength(true);
  TwoViewGeometry::Options options;
  options.ransac_options.max_error = 1.0;
  options.compute_relative_pose = true;  // Enable relative pose estimation.
  TwoViewGeometry two_view_geometry;
  two_view_geometry.Estimate(cam1, pts1, cam2, pts2, matches, options);

  // Output EST.
  std::cout << "ET Pose:\n"
            << two_view_geometry.qvec << "\n"
            << two_view_geometry.tvec << "\n";
  std::cout << "ET E: \n"
            << two_view_geometry.E << "\n"
            << "ET F: " << two_view_geometry.F / two_view_geometry.F(2, 2)
            << std::endl;

  std::cout << "test quaternion: "
            << Eigen::Quaterniond(
                   Eigen::AngleAxisd(-0.05 * M_PI, Eigen::Vector3d::UnitY()))
                   .coeffs()
            << std::endl;

  // Check result.
  BOOST_CHECK_EQUAL(two_view_geometry.config, TwoViewGeometry::CALIBRATED);
  BOOST_CHECK(two_view_geometry.F.isApprox(F));
  BOOST_CHECK(two_view_geometry.E.isApprox(E));
  // BOOST_CHECK(two_view_geometry.H.isApprox(H));
  BOOST_CHECK(two_view_geometry.qvec.isApprox(RotationMatrixToQuaternion(R)));
  BOOST_CHECK(two_view_geometry.tvec.isApprox(t));
}
