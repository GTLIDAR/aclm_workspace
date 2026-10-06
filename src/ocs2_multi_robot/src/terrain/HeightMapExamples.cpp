#include "ocs2_multi_robot/terrain/HeightMapExamples.h"

namespace ocs2 {
namespace multi_robot {

FlatGround::FlatGround(double height)
{
  height_ = height;
}

double
Block::GetHeight (double x, double y) const
{
  double h = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <=block_start+eps_)
    h = slope_*(x-block_start);

  if (block_start+eps_ <= x && x <= block_start+length_)
    h = height_;

  return h;
}

double
Block::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <=block_start+eps_)
    dhdx = slope_;

  return dhdx;
}


// STAIRS
double
Stairs::GetHeight (double x, double y) const
{
  double h = 0.0;

  if (x>=first_step_start_)
    h = height_first_step;

  if (x>=first_step_start_+first_step_width_)
    h = height_second_step;

  if (x>=first_step_start_+first_step_width_+width_top)
    h = 0.0;

  return h;
}


// GAP
double
Gap::GetHeight (double x, double y) const
{
  double h = 0.0;

  // modelled as parabola
  if (gap_start_ <= x && x <= gap_end_x)
    h = a*x*x + b*x + c;

  return h;
}

double
Gap::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dhdx = 2*a*x + b;

  return dhdx;
}

double
Gap::GetHeightDerivWrtXX (double x, double y) const
{
  double dzdxx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dzdxx = 2*a;

  return dzdxx;
}


// SLOPE
double
Slope::GetHeight (double x, double y) const
{
  double z = 0.0;
  if (x >= slope_start_)
    z = slope_*(x-slope_start_);

  // going back down
  if (x >= x_down_start_) {
    z = height_center - slope_*(x-x_down_start_);
  }

  // back on flat ground
  if (x >= x_flat_start_)
    z = 0.0;

  return z;
}

double
Slope::GetHeightDerivWrtX (double x, double y) const
{
  double dzdx = 0.0;
  if (x >= slope_start_)
    dzdx = slope_;

  if (x >= x_down_start_)
    dzdx = -slope_;

  if (x >= x_flat_start_)
    dzdx = 0.0;

  return dzdx;
}


// Chimney
double
Chimney::GetHeight (double x, double y) const
{
  double z = 0.0;

  if (x_start_<=x && x<=x_end_)
    z = slope_*(y-y_start_);

  return z;
}

double
Chimney::GetHeightDerivWrtY (double x, double y) const
{
  double dzdy = 0.0;

  if (x_start_<= x && x<= x_end_)
    dzdy = slope_;

  return dzdy;
}


// Chimney LR
double
ChimneyLR::GetHeight (double x, double y) const
{
  double z = 0.0;

  if (x_start_<=x && x<=x_end1_)
    z = slope_*(y-y_start_);

  if (x_end1_<=x && x<=x_end2_)
    z = -slope_*(y+y_start_);

  return z;
}

double
ChimneyLR::GetHeightDerivWrtY (double x, double y) const
{
  double dzdy = 0.0;

  if (x_start_ <= x && x <= x_end1_)
    dzdy = slope_;

  if (x_end1_<=x && x<=x_end2_)
    dzdy = -slope_;

  return dzdy;
}

double Hill::GetHeight(double x, double y) const {
    double h = -factor_x_ * pow(x - center_x_, 2.0)
               - factor_y_ * pow(y - center_y_, 2.0);

    return h;
}

double Hill::GetHeightDerivWrtX(double x, double) const {
    double dhdx = -2.0 * factor_x_ * (x - center_x_);

    return dhdx;
}

double Hill::GetHeightDerivWrtY(double, double y) const {
    double dhdy = -2.0 * factor_y_ * (y - center_y_);

    return dhdy;
}

double Hill::GetHeightDerivWrtXX(double, double) const {
    return -2.0 * factor_x_;
}

double Hill::GetHeightDerivWrtYY(double, double) const {
    return -2.0 * factor_y_;
}

HeightMap::Ptr HeightMap::MakeTerrain(TerrainID type) {
  switch (type) {
    case FlatID:
      return std::make_shared<FlatGround>();
      break;
    case BlockID:
      return std::make_shared<Block>();
      break;
    case StairsID:
      return std::make_shared<Stairs>();
      break;
    case GapID:
      return std::make_shared<Gap>();
      break;
    case SlopeID:
      return std::make_shared<Slope>();
      break;
    case ChimneyID:
      return std::make_shared<Chimney>();
      break;
    case ChimneyLRID:
      return std::make_shared<ChimneyLR>();
      break;
    case HillID:
      return std::make_shared<Hill>();
      break;
//            case SlopeConstantID:
//                return std::make_shared<SlopeConstant>();
//                break;
    default:
        assert(false && "Terrain not defined");
        break;
  }

  return nullptr;
}

} // namespace multi_robot
} // namespace ocs2