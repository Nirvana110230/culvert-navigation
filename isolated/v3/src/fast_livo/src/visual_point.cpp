/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "visual_point.h"
#include "feature.h"
#include <stdexcept>
#include <vikit/math_utils.h>

VisualPoint::VisualPoint(const Vector3d &pos)
    : pos_(pos), normal_(Vector3d::Zero()), normal_information_(Matrix3d::Zero()),
      previous_normal_(Vector3d::Zero()), covariance_(Matrix3d::Zero()),
      is_converged_(false), is_normal_initialized_(false), has_ref_patch_(false)
{
  lru_id_ = ++memory_diagnostics::points_created;
}

VisualPoint::~VisualPoint() 
{
  ++memory_diagnostics::points_deleted;
  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    delete(*it);
  }
  obs_.clear();
  ref_patch = nullptr;
}

void VisualPoint::addFrameRef(Feature *ftr)
{
  if (ftr) obs_.push_front(ftr);
}

void VisualPoint::deleteFeatureRef(Feature *ftr)
{
  if (ref_patch == ftr)
  {
    ref_patch = nullptr;
    has_ref_patch_ = false;
  }
  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    if ((*it) == ftr)
    {
      delete((*it));
      obs_.erase(it);
      return;
    }
  }
}

bool VisualPoint::getCloseViewObs(const Vector3d &framepos, Feature *&ftr, const Vector2d &cur_px) const
{
  ftr = nullptr;
  const Vector3d obs_dir = (framepos - pos_).normalized();
  double best_cos = 0.5; // At most 60 degrees from the current view.
  for (auto* candidate : obs_)
  {
    if (!candidate || !candidate->hasImagePatch()) continue;
    const Vector3d dir = (candidate->pos() - pos_).normalized();
    const double cos_angle = obs_dir.dot(dir);
    if (cos_angle >= 0.5 && (!ftr || cos_angle > best_cos))
    {
      best_cos = cos_angle;
      ftr = candidate;
    }
  }
  return ftr != nullptr;
}

void VisualPoint::findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const
{
  ftr = nullptr;
  float min_score = std::numeric_limits<float>::max();

  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    if (*it && (*it)->score_ < min_score)
    {
      min_score = (*it)->score_;
      ftr = *it;
    }
  }
}

void VisualPoint::deleteNonRefPatchFeatures()
{
  if (!ref_patch) return;
  for (auto it = obs_.begin(); it != obs_.end();)
  {
    if (*it != ref_patch)
    {
      delete *it;
      it = obs_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}
