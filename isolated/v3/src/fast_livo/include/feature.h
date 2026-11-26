/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_FEATURE_H_
#define LIVO_FEATURE_H_

#include "visual_point.h"
#include "memory_diagnostics.h"
#include <limits>

// A salient image region that is tracked across frames.
struct Feature
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum FeatureType
  {
    CORNER,
    EDGELET
  };
  std::uint64_t last_used_frame_ = 0, lru_id_ = 0;
  int id_ = -1;
  FeatureType type_;     //!< Type can be corner or edgelet.
  cv::Mat img_;          //!< Image associated with the patch feature
  Vector2d px_;          //!< Coordinates in pixels on pyramid level 0.
  Vector3d f_;           //!< Unit-bearing vector of the patch feature.
  int level_;            //!< Image pyramid level where patch feature was extracted.
  VisualPoint *point_;   //!< Pointer to 3D point which corresponds to the patch feature.
  Vector2d grad_ = Vector2d::Zero(); //!< Dominant gradient direction for edglets, normalized.
  SE3<double> T_f_w_;            //!< Pose of the frame where the patch feature was extracted.
  float *patch_;         //!< Pointer to the image patch data.
  float score_ = 0;
  double mean_ = std::numeric_limits<double>::quiet_NaN(); //!< Cached mean of immutable patch_.
  double inv_expo_time_ = 1.0;
  
  Feature(VisualPoint *_point, float *_patch, const Vector2d &_px, const Vector3d &_f, const SE3<double> &_T_f_w, int _level)
      : type_(CORNER), px_(_px), f_(_f), level_(_level), point_(_point), T_f_w_(_T_f_w), patch_(_patch)
  {
    lru_id_ = ++memory_diagnostics::features_created;
  }

  inline Vector3d pos() const { return T_f_w_.inverse().translation(); }
  bool hasImagePatch() const { return patch_ && !img_.empty() && img_.type() == CV_8UC1; }
  Feature(const Feature&) = delete;
  Feature& operator=(const Feature&) = delete;
  
  ~Feature()
  {
    // ROS_WARN("The feature %d has been destructed.", id_);
    memory_diagnostics::release(img_);
    ++memory_diagnostics::features_deleted;
    delete[] patch_;
  }
};

#endif // LIVO_FEATURE_H_
