#include "orion_mtc/perception/target_selector.hpp"
#include <cmath>
#include <limits>

namespace orion_mtc
{

TargetSelector::TargetSelector(const TargetSelectorParams& params) : params_(params)
{
}

std::optional<TargetObject> TargetSelector::select(const std::vector<TargetObject>& targets) const
{
  if (targets.empty())
  {
    return std::nullopt;
  }
  double best_score = -std::numeric_limits<double>::max();
  std::optional<TargetObject> best;
  const double nx = params_.nominal_x;
  const double ny = params_.nominal_y;
  const double nz = params_.nominal_z;
  const double w_dist = params_.distance_weight;
  const double w_align = params_.alignment_weight;
  const double min_conf = params_.min_confidence;

  for (const auto& t : targets)
  {
    if (t.confidence < min_conf)
    {
      continue;
    }
    double dx = t.position.x - nx;
    double dy = t.position.y - ny;
    double dz = t.position.z - nz;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < 1e-9)
    {
      dist = 1e-9;
    }
    double distance_score = -dist;
    double alignment_score = t.axis_direction.z;
    if (alignment_score < -1.0)
    {
      alignment_score = -1.0;
    }
    if (alignment_score > 1.0)
    {
      alignment_score = 1.0;
    }
    double score = w_dist * distance_score + w_align * alignment_score;
    if (score > best_score)
    {
      best_score = score;
      best = t;
    }
  }
  return best;
}

void TargetSelector::setParams(const TargetSelectorParams& params)
{
  params_ = params;
}

}  // namespace orion_mtc
