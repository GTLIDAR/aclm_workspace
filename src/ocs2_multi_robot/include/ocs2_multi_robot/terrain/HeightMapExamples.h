#pragma once

#include "ocs2_multi_robot/terrain/HeightMap.h"

namespace ocs2 {
namespace multi_robot {
 

/**
 * @addtogroup Terrains
 * @{
 */

/**
 * @brief Sample terrain of even height.
 */
class FlatGround : public HeightMap {
public:
  FlatGround(double height = 0.0);

  double GetHeight(double x, double y) const override { return height_; };

private:
  double height_; // [m]
};

/**
 * @brief Sample terrain with a step in height in x-direction.
 */
class Block : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

  double GetHeightDerivWrtX(double x, double y) const override;

private:
  double block_start = 0.4;
  double length_ = 0.2;
  double height_ = 0.2; // [m]

  double eps_ = 0.0005; // approximate as slope
  const double slope_ = height_ / eps_;
};

/**
 * @brief Sample terrain with a two-steps in height in x-direction.
 */
class Stairs : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

private:
  double first_step_start_ = 1.0;
  double first_step_width_ = 0.4;
  double height_first_step = 0.2;
  double height_second_step = 0.4;
  double width_top = 1.0;
};

/**
 * @brief Sample terrain with parabola-modeled gap in x-direction.
 */
class Gap : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

  double GetHeightDerivWrtX(double x, double y) const override;

  double GetHeightDerivWrtXX(double x, double y) const override;

private:
  const double gap_start_ = 0.6;
  const double w = 0.3;
  const double h = 1.5;

  const double slope_ = h / w;
  const double dx = w / 2.0;
  const double xc = gap_start_ + dx; // gap center
  const double gap_end_x = gap_start_ + w;

  // generated with matlab
  // see matlab/gap_height_map.m
  // coefficients of 2nd order polynomial
  // h = a*x^2 + b*x + c
  const double a = (4 * h) / (w * w);
  const double b = -(8 * h * xc) / (w * w);
  const double c = -(h * (w - 2 * xc) * (w + 2 * xc)) / (w * w);
};

/**
 * @brief Sample terrain with an increasing and then decreasing slope in x-direction.
 */
class Slope : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

  double GetHeightDerivWrtX(double x, double y) const override;

private:
  const double slope_start_ = 0.1;
  const double up_length_ = 3.0;
  const double down_length_ = 3.0;
  const double height_center = 1.5;

  const double x_down_start_ = slope_start_ + up_length_;
  const double x_flat_start_ = x_down_start_ + down_length_;
  const double slope_ = height_center / up_length_;
};

/**
 * @brief Sample terrain with a tilted vertical wall to cross a gap.
 */
class Chimney : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

  double GetHeightDerivWrtY(double x, double y) const override;

private:
  const double x_start_ = 1.0;
  const double length_ = 1.5;
  const double y_start_ = 0.5; // distance to start of slope from center at z=0
  const double slope_ = 3.0;

  const double x_end_ = x_start_ + length_;
};

/**
 * @brief Sample terrain with two tilted vertical walls to cross a gap.
 */
class ChimneyLR : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

  double GetHeightDerivWrtY(double x, double y) const override;

private:
  const double x_start_ = 0.5;
  const double length_ = 0.5;
  const double y_start_ = 0.5; // distance to start of slope from center at z=0
  const double slope_ = 0.4;

  const double x_end1_ = x_start_ + length_;
  const double x_end2_ = x_start_ + 2 * length_;
};

/**
 * @brief Sample terrain which is a quadratic hill
 */
class Hill : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

  double GetHeightDerivWrtX(double x, double y) const override;

  double GetHeightDerivWrtY(double x, double y) const override;

  double GetHeightDerivWrtXX(double x, double y) const override;

  double GetHeightDerivWrtYY(double x, double y) const override;

private:
  double center_x_ = -1.0;
  double center_y_ = -1.0;
  double factor_x_ = 0.1;
  double factor_y_ = 0.1;
};

} // namespace multi_robot
} // namespace ocs2