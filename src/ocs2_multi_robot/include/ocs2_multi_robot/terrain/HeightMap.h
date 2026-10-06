#pragma once

#include <memory>
#include <vector>
#include <map>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "ocs2_multi_robot/common/Types.h"
#include <unordered_map>

namespace ocs2 {
namespace multi_robot {

/**
 * @defgroup Terrains Terrains
 * @brief The heightmap and slope of various terrains.
 */

/**
 * @brief Holds the height and slope information of the terrain.
 *
 * This class and the examples were taken from TOWR!
 *
 * This class is responsible for providing the height values and slope at
 * each position (x,y). Examples of various height map examples can be found
 * in height_map_examples.h.
 *
 * If a height map of the terrain already exists, e.g. Octomap/Gridmap, then
 * a simple adapter for these can be written to comply to  this minimal
 * interface and to be used with %towr.
 *
 * The height map is used to formulate constraints such as
 * "foot must be touching terrain during stance phase".
 * @sa TerrainConstraint
 *
 * @ingroup Terrains
 */
class HeightMap {
public:
  using Ptr = std::shared_ptr<HeightMap>;
  using Vector3d = Eigen::Vector3d;
  using MatrixXd = Eigen::MatrixXd;

  /**
   * @brief Terrains IDs corresponding for factory method.
   */
  enum TerrainID {
    FlatID,
    BlockID,
    StairsID,
    GapID,
    SlopeID,
    ChimneyID,
    ChimneyLRID,
    HillID,
    SlopeConstantID,
    TERRAIN_COUNT
  };

  enum Direction {
    Normal, Tangent1, Tangent2
  };

  /** @brief Instantiate one of the sample terrains */
  static HeightMap::Ptr MakeTerrain(TerrainID type);

  HeightMap() = default;

  virtual ~HeightMap() = default;

  /**
   * @returns The height of the terrain [m] at a specific 2D position.
   * @param x The x position.
   * @param y The y position.
   */
  virtual double GetHeight(double x, double y) const = 0;

  double GetHeight(const Vector3d& pos) const;

  /**
   * @brief How the height value changes at a 2D position in direction dim.
   * @param dim  The direction (x,y) w.r.t. which the height change is desired.
   * @param x  The x position on the terrain.
   * @param y  The y position on the terrain.
   * @return  The derivative of the height with respect to the dimension.
   */
  double GetDerivativeOfHeightWrt(Dim2D dim, double x, double y) const;

  double GetDerivativeOfHeightWrt(Dim2D dim, const Vector3d& pos) const;

  /**
   * @brief Returns either the vector normal or tangent to the terrain patch.
   * @param direction  The terrain normal or tangent vectors.
   * @param x  The x position on the terrain.
   * @param y  The y position on the terrain.
   * @return The normalized 3D vector in the specified direction.
   */
  Vector3d GetNormalizedBasis(Direction direction, double x, double y) const;

  Vector3d GetNormalizedBasis(Direction direction, const Vector3d& pos) const;

  /**
   * @brief How the terrain normal/tangent vectors change when moving in x or y.
   * @param direction  The terrain normal or tangent vectors.
   * @param dim  The dimension w.r.t which the change is searched for.
   * @param x  The x position on the terrain.
   * @param y  The y position on the terrain.
   * @return The normalized 3D derivative w.r.t dimension dim.
   */
  Vector3d GetDerivativeOfNormalizedBasisWrt(Direction direction, Dim2D dim,
                                              double x, double y) const;

  Vector3d GetDerivativeOfNormalizedBasisWrt(Direction direction, Dim2D dim,
                                              const Vector3d& pos) const;

  /**
   * @brief How the terrain normal/tangent vectors change when moving
   *
   * This version returns an entire jacobian. The w.r.t.-z component is zero.
   *
   * @param direction  The terrain normal or tangent vectors.
   * @param pos  The x, y and z position on the terrain.
   * @return The normalized 3D derivative w.r.t dimension dim.
   */
  MatrixXd GetDerivativeOfNormalizedBasis(Direction direction,
                                          const Vector3d& pos) const;

  /**
   * @returns The constant friction coefficient over the whole terrain.
   */
  double GetFrictionCoeff() const;

protected:
  double friction_coeff_ = 0.5;

private:
  using DimDerivs = std::vector<Dim2D>; ///< dimensional derivatives
  /**
   * @brief returns either the terrain normal/tangent or its derivative.
   * @param direction Terrain normal or tangent vector.
   * @param x The x position on the terrain.
   * @param y The y position on the terrain.
   * @param dim_deriv If empty, the vector is returned, if e.g. X_ is set, the
   *                  derivative of the vector w.r.t. x is returned.
   * @returns the 3D @b not-normalized vector.
   */
  Vector3d GetBasis(Direction direction, double x, double y,
                    const DimDerivs& dim_deriv = {}) const;

  Vector3d GetNormal(double x, double y, const DimDerivs& = {}) const;

  Vector3d GetTangent1(double x, double y, const DimDerivs& = {}) const;

  Vector3d GetTangent2(double x, double y, const DimDerivs& = {}) const;

  double GetSecondDerivativeOfHeightWrt(Dim2D dim1, Dim2D dim2, double x,
                                        double y) const;

  Vector3d GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex(
          const Vector3d& non_normalized, int index) const;

  MatrixXd GetDerivativeOfBasis(Direction direction,
                                const Vector3d& pos) const;

  MatrixXd GetDerivativeOfNormalizedVectorWrtNonNormalized(
          const Vector3d& v) const;

  // first derivatives that must be implemented by the user
  virtual double GetHeightDerivWrtX(double x, double y) const;

  virtual double GetHeightDerivWrtY(double x, double y) const;

  // second derivatives with respect to first letter, then second
  virtual double GetHeightDerivWrtXX(double x, double y) const;

  virtual double GetHeightDerivWrtXY(double x, double y) const;

  virtual double GetHeightDerivWrtYX(double x, double y) const;

  virtual double GetHeightDerivWrtYY(double x, double y) const;
};

const static std::map<HeightMap::TerrainID, std::string> terrain_names = {
  {HeightMap::FlatID,      "Flat"},
  {HeightMap::BlockID,     "Block"},
  {HeightMap::StairsID,    "Stairs"},
  {HeightMap::GapID,       "Gap"},
  {HeightMap::SlopeID,     "Slope"},
  {HeightMap::ChimneyID,   "Chimney"},
  {HeightMap::ChimneyLRID, "ChimenyLR"},
};

static std::unordered_map<std::string, HeightMap::TerrainID> terrain_mapping = {
  {"Flat", HeightMap::TerrainID::FlatID},
  {"Block", HeightMap::TerrainID::BlockID},
  {"Stairs", HeightMap::TerrainID::StairsID},
  {"Gap", HeightMap::TerrainID::GapID},
  {"Slope", HeightMap::TerrainID::SlopeID},
  {"Chimney", HeightMap::TerrainID::ChimneyID},
  {"ChimneyLR", HeightMap::TerrainID::ChimneyLRID},
  {"Hill", HeightMap::TerrainID::HillID}
};

} // namespace multi_robot
} // namespace ocs2