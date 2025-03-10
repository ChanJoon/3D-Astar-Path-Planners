#pragma once

#include "bonxai.hpp"
#include <Eigen/Geometry>
#include <unordered_set>

namespace Bonxai
{

  template <class Functor>
  void RayIterator(const CoordT& key_origin,
                  const CoordT& key_end,
                  const Functor& func);

  inline void ComputeRay(const CoordT& key_origin,
                        const CoordT& key_end,
                        std::vector<CoordT>& ray)
  {
    ray.clear();
    RayIterator(key_origin, key_end, [&ray](const CoordT& coord)
    {
      ray.push_back(coord);
      return true;
    });
  }

  /**
   * @brief The ProbabilisticMap class is meant to behave as much as possible as
   * octomap::Octree, given the same voxel size.
   *
   * Insert a point cloud to update the current probability
   */
  class ProbabilisticMap
  {
  public:
    using Vector3D = Eigen::Vector3d;

    /// Compute the logds, but return the result as an integer,
    /// The real number is represented as a fixed precision
    /// integer (6 decimals after the comma)
    [[nodiscard]] static constexpr int32_t logods(float prob)
    {
      return int32_t(1e6 * std::log(prob / (1.0 - prob)));
    }
    /// Expect the fixed comma value returned by logods()
    [[nodiscard]] static constexpr float prob(int32_t logods_fixed)
    {
      float logods = float(logods_fixed) * 1e-6;
      return (1.0 - 1.0 / (1.0 + std::exp(logods)));
    }

    struct CellT
    {
      // variable used to check if a cell was already updated in this loop
      int32_t update_id : 4;
      // the probability of the cell to be occupied
      int32_t probability_log : 28;
      // the previous probability of the cell to be occupied for tracking change
      int32_t prev_probability_occupied_log : 28;
      // the previous probability of the cell to be occupied for tracking change
      int32_t prev_probability_free_log : 28;

      CellT()
        : update_id(0)
        , probability_log(UnknownProbability)
        , prev_probability_occupied_log(UnknownProbability)
        , prev_probability_free_log(UnknownProbability){};
    };

    /// These default values are the same as OctoMap
    struct Options
    {
      int32_t prob_miss_log = logods(0.45f);
      int32_t prob_hit_log = logods(0.65f);

      int32_t clamp_min_log = logods(0.12f);
      int32_t clamp_max_log = logods(0.97f);

      int32_t occupancy_threshold_log = logods(0.5);
    };

    static const int32_t UnknownProbability;

    ProbabilisticMap(const double& resolution);

    void Clear() { _grid.Clear(); }

    [[nodiscard]] VoxelGrid<CellT>& grid();
    [[nodiscard]] const VoxelGrid<CellT>& grid() const;

    [[nodiscard]] const Options& options() const;
    void setOptions(const Options& options);

    /**
     * @brief insertPointCloud will update the probability map
     * with a new set of detections.
     * The template function can accept points of different types,
     * such as pcl:Point, Eigen::Vector or Bonxai::Point3d
     *
     * Both origin and points must be in word coordinates
     *
     * @param points   a vector of points which represent detected obstacles
     * @param origin   origin of the point cloud
     * @param max_range  max range of the ray, if exceeded, we will use that
     * to compute a free space
     */
    template <typename PointT, typename Allocator>
    void insertPointCloud(const std::vector<PointT, Allocator>& points,
                          const PointT& origin,
                          const double& max_range);
    // This function is usually called by insertPointCloud
    // We expose it here to add more control to the user.
    // Once finished adding points, you must call updateFreeCells()
    void addHitPoint(const Vector3D& point);
    // This function is usually called by insertPointCloud
    // We expose it here to add more control to the user.
    // Once finished adding points, you must call updateFreeCells()
    void addMissPoint(const Vector3D& point);

    /**
     * @brief raycast along the direction around the collision radius from the origin
     */
    template <typename PointT>
    void rayCastPointRadius(const PointT& origin_pt, const PointT& direction, const double& max_range, const double& collision_radius, std::vector<PointT>& casted_points, double& valid_ratio);

    [[nodiscard]] bool isOccupied(const CoordT& coord) const;
    [[nodiscard]] bool isUnknown(const CoordT& coord) const;
    [[nodiscard]] bool isFree(const CoordT& coord) const;

    void getOccupiedVoxels(std::vector<CoordT>& coords);
    void getFreeVoxels(std::vector<CoordT>& coords);
    void getNewFreeVoxelsAndReset(std::vector<CoordT>& coords);
    void getNewOccupiedVoxelsAndReset(std::vector<CoordT>& coords);
    double getNearestOccupiedDistance(const Vector3D& point);

    template <typename PointT>
    void getOccupiedVoxels(std::vector<PointT>& points)
    {
      thread_local std::vector<CoordT> coords;
      coords.clear();
      getOccupiedVoxels(coords);
      for (const auto& coord : coords)
      {
        const auto p = _grid.coordToPos(coord);
        points.emplace_back(p.x, p.y, p.z);
      }
      return;
    }
    template <typename PointT>
    void getFreeVoxels(std::vector<PointT>& points)
    {
      thread_local std::vector<CoordT> coords;
      coords.clear();
      getFreeVoxels(coords);
      for (const auto& coord : coords)
      {
        const auto p = _grid.coordToPos(coord);
        points.emplace_back(p.x, p.y, p.z);
      }
      return;
    }
    template <typename PointT>
    void getNewFreeVoxelsAndReset(std::vector<PointT>& points)
    {
      thread_local std::vector<CoordT> coords;
      coords.clear();
      getNewFreeVoxelsAndReset(coords);
      for (const auto& coord : coords)
      {
        const auto p = _grid.coordToPos(coord);
        points.emplace_back(p.x, p.y, p.z);
      }
      return;
    }
    template <typename PointT>
    void getNewFreeVoxelsAndReset(std::vector<PointT>& points, std::vector<CoordT>& coords)
    {
      coords.clear();
      getNewFreeVoxelsAndReset(coords);
      for (const auto& coord : coords)
      {
        const auto p = _grid.coordToPos(coord);
        points.emplace_back(p.x, p.y, p.z);
      }
      return;
    }
    template <typename PointT>
    void getNewOccupiedVoxelsAndReset(std::vector<PointT>& points)
    {
      thread_local std::vector<CoordT> coords;
      coords.clear();
      getNewOccupiedVoxelsAndReset(coords);
      for (const auto& coord : coords)
      {
        const auto p = _grid.coordToPos(coord);
        points.emplace_back(p.x, p.y, p.z);
      }
      return;
    }

  private:
    VoxelGrid<CellT> _grid;
    Options _options;
    uint8_t _update_count = 1;

    std::vector<CoordT> _miss_coords;
    std::vector<CoordT> _hit_coords;

    mutable Bonxai::VoxelGrid<CellT>::Accessor _accessor;

    void updateFreeCells(const Vector3D& origin);
    template <typename PointT>
    bool rayCastPoint(const CoordT& origin, const CoordT& end, PointT& casted_point);
  };

  //--------------------------------------------------

  template <typename PointT, typename Alloc>
  inline void ProbabilisticMap::insertPointCloud(const std::vector<PointT, Alloc>& points,
                                                const PointT& origin,
                                                const double& max_range)
  {
    const auto from = ConvertPoint<Vector3D>(origin);
    const double max_range_sqr = max_range * max_range;
    for (const auto& point : points)
    {
      const auto to = ConvertPoint<Vector3D>(point);
      Vector3D vect(to - from);
      const double squared_norm = vect.squaredNorm();
      // points that exceed the max_range will create a cleaning ray
      if (squared_norm >= max_range_sqr)
      {
        // The new point will have distance == max_range from origin
        const Vector3D new_point = from + ((vect / std::sqrt(squared_norm)) * max_range);
        addMissPoint(new_point);
      }
      else {
        addHitPoint(to);
      }
    }
    updateFreeCells(from);
  }

  template <class Functor> inline
  void RayIterator(const CoordT& key_origin,
                  const CoordT& key_end,
                  const Functor &func)
  {
    if (key_origin == key_end)
    {
      return;
    }
    if(!func(key_origin))
    {
      return;
    }

    CoordT error = { 0, 0, 0 };
    CoordT coord = key_origin;
    CoordT delta = (key_end - coord);
    const CoordT step = { delta.x < 0 ? -1 : 1,
                          delta.y < 0 ? -1 : 1,
                          delta.z < 0 ? -1 : 1 };

    delta = { delta.x < 0 ? -delta.x : delta.x,
              delta.y < 0 ? -delta.y : delta.y,
              delta.z < 0 ? -delta.z : delta.z };

    const int max = std::max(std::max(delta.x, delta.y), delta.z);

    // maximum change of any coordinate
    for (int i = 0; i < max - 1; ++i)
    {
      // update errors
      error = error + delta;
      // manual loop unrolling
      if ((error.x << 1) >= max)
      {
        coord.x += step.x;
        error.x -= max;
      }
      if ((error.y << 1) >= max)
      {
        coord.y += step.y;
        error.y -= max;
      }
      if ((error.z << 1) >= max)
      {
        coord.z += step.z;
        error.z -= max;
      }
      if(!func(coord))
      {
        return;
      }
    }
    return;
  }

}  // namespace Bonxai





namespace Bonxai
{
  inline const int32_t ProbabilisticMap::UnknownProbability = ProbabilisticMap::logods(0.5f);

  inline VoxelGrid<ProbabilisticMap::CellT>& ProbabilisticMap::grid()
  {
    return _grid;
  }

  inline ProbabilisticMap::ProbabilisticMap(const double& resolution)
    : _grid(resolution)
    , _accessor(_grid.createAccessor())
  {}

  inline const VoxelGrid<ProbabilisticMap::CellT>& ProbabilisticMap::grid() const
  {
    return _grid;
  }

  inline const ProbabilisticMap::Options& ProbabilisticMap::options() const
  {
    return _options;
  }

  inline void ProbabilisticMap::setOptions(const Options& options)
  {
    _options = options;
    return;
  }

  inline void ProbabilisticMap::addHitPoint(const Vector3D &point)
  {
    const auto coord = _grid.posToCoord(point);
    CellT* cell = _accessor.value(coord, true);

    if (cell->update_id != _update_count)
    {
      cell->probability_log = std::min(cell->probability_log + _options.prob_hit_log,
                                      _options.clamp_max_log);
      cell->prev_probability_free_log = cell->probability_log;

      cell->update_id = _update_count;
      _hit_coords.push_back(coord);
    }
    return;
  }

  inline void ProbabilisticMap::addMissPoint(const Vector3D &point)
  {
    const auto coord = _grid.posToCoord(point);
    CellT* cell = _accessor.value(coord, true);

    if (cell->update_id != _update_count)
    {
      cell->probability_log = std::max(
          cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);
      cell->prev_probability_occupied_log = cell->probability_log;

      cell->update_id = _update_count;
      _miss_coords.push_back(coord);
    }
    return;
  }

  inline bool ProbabilisticMap::isOccupied(const CoordT &coord) const
  {
    if(auto* cell = _accessor.value(coord, false))
    {
      return cell->probability_log > _options.occupancy_threshold_log;
    }
    return false;
  }

  inline bool ProbabilisticMap::isUnknown(const CoordT &coord) const
  {
    if(auto* cell = _accessor.value(coord, false))
    {
      return cell->probability_log == _options.occupancy_threshold_log;
    }
    return true;
  }

  inline bool ProbabilisticMap::isFree(const CoordT &coord) const
  {
    if(auto* cell = _accessor.value(coord, false))
    {
      return cell->probability_log < _options.occupancy_threshold_log;
    }
    return false;
  }

  inline void Bonxai::ProbabilisticMap::updateFreeCells(const Vector3D& origin)
  {
    auto accessor = _grid.createAccessor();

    // same as addMissPoint, but using lambda will force inlining
    auto clearPoint = [this, &accessor](const CoordT& coord)
    {
      CellT* cell = accessor.value(coord, true);
      if (cell->update_id != _update_count)
      {
        cell->probability_log = std::max(
            cell->probability_log + _options.prob_miss_log, _options.clamp_min_log);
        cell->prev_probability_occupied_log = cell->probability_log;
        cell->update_id = _update_count;
      }
      return true;
    };

    const auto coord_origin = _grid.posToCoord(origin);

    for (const auto& coord_end : _hit_coords)
    {
      RayIterator(coord_origin, coord_end, clearPoint);
    }
    _hit_coords.clear();

    for (const auto& coord_end : _miss_coords)
    {
      RayIterator(coord_origin, coord_end, clearPoint);
    }
    _miss_coords.clear();

    if (++_update_count == 4)
    {
      _update_count = 1;
    }
    return;
  }

  inline void ProbabilisticMap::getOccupiedVoxels(std::vector<CoordT>& coords)
  {
    coords.clear();
    auto visitor = [&](CellT& cell, const CoordT& coord) {
      if (cell.probability_log > _options.occupancy_threshold_log)
      {
        coords.push_back(coord);
      }
    };
    _grid.forEachCell(visitor);
    return;
  }

  inline void ProbabilisticMap::getFreeVoxels(std::vector<CoordT>& coords)
  {
    coords.clear();
    auto visitor = [&](CellT& cell, const CoordT& coord) {
      if (cell.probability_log < _options.occupancy_threshold_log)
      {
        coords.push_back(coord);
      }
    };
    _grid.forEachCell(visitor);
    return;
  }

  inline void ProbabilisticMap::getNewFreeVoxelsAndReset(std::vector<CoordT>& coords)
  {
    coords.clear();
    auto visitor = [&](CellT& cell, const CoordT& coord) {
      if (cell.probability_log < _options.occupancy_threshold_log) // now free
      {
        if (cell.prev_probability_free_log >= _options.occupancy_threshold_log) // but, not free before
        {
          cell.prev_probability_free_log = cell.probability_log;
          coords.push_back(coord);
        }
      }
    };
    _grid.forEachCell(visitor);
    return;
  }

  inline void ProbabilisticMap::getNewOccupiedVoxelsAndReset(std::vector<CoordT>& coords)
  {
    coords.clear();
    auto visitor = [&](CellT& cell, const CoordT& coord) {
      if (cell.probability_log > _options.occupancy_threshold_log) // now occupied
      {
        if (cell.prev_probability_occupied_log <= _options.occupancy_threshold_log) // but, not occupied before
        {
          cell.prev_probability_occupied_log = cell.probability_log;
          coords.push_back(coord);
        }
      }
    };
    _grid.forEachCell(visitor);
    return;
  }

  template <typename PointT>
  inline bool ProbabilisticMap::rayCastPoint(const CoordT& origin, const CoordT& end, PointT& casted_point)
  {
    if (origin == end)
    {
      return false;
    }

    CoordT error = { 0, 0, 0 };
    CoordT delta = (end - origin);
    CoordT coord = origin;
    const CoordT step = { delta.x < 0 ? -1 : 1,
                          delta.y < 0 ? -1 : 1,
                          delta.z < 0 ? -1 : 1 };
    delta = { delta.x < 0 ? -delta.x : delta.x,
              delta.y < 0 ? -delta.y : delta.y,
              delta.z < 0 ? -delta.z : delta.z };

    // maximum change of any coordinate
    const int max = std::max(std::max(delta.x, delta.y), delta.z);
    for (int i = 0; i < max - 1; ++i)
    {
      // update errors
      error = error + delta;
      // manual loop unrolling
      if ((error.x << 1) >= max)
      {
        coord.x += step.x;
        error.x -= max;
      }
      if ((error.y << 1) >= max)
      {
        coord.y += step.y;
        error.y -= max;
      }
      if ((error.z << 1) >= max)
      {
        coord.z += step.z;
        error.z -= max;
      }
      if(isOccupied(coord))
      {
        casted_point = ConvertPoint<PointT>(_grid.coordToPos(coord));
        return true;
      }
    }
    return false;
  }

  template <typename PointT>
  inline void ProbabilisticMap::rayCastPointRadius(const PointT& origin_pt, const PointT& direction, const double& max_range, const double& collision_radius, std::vector<PointT>& casted_points, double& valid_ratio)
  {
    std::vector<PointT>().swap(casted_points);
    const Vector3D origin_vec3d_ = ConvertPoint<Vector3D>(origin_pt);
    const CoordT origin_coord_ = _grid.posToCoord(origin_vec3d_);
    const Vector3D direction_vec3d_ = ConvertPoint<Vector3D>(direction).normalized() * max_range;
    const CoordT direction_coord_ = _grid.posToCoord(direction_vec3d_);
    const Vector3D min_bound_ = origin_vec3d_.array() - collision_radius;
    const Vector3D max_bound_ = origin_vec3d_.array() + collision_radius;
    const CoordT min_bound_coord_ = _grid.posToCoord(min_bound_);
    const CoordT max_bound_coord_ = _grid.posToCoord(max_bound_);

    int counter_ = 0;
    for (int32_t i = min_bound_coord_.x; i < max_bound_coord_.x+1; i++)
    {
      for (int32_t j = min_bound_coord_.y; j < max_bound_coord_.y+1; j++)
      {
        counter_++;
        PointT casted_point_;
        CoordT radius_origin_coord_ = { i, j, origin_coord_.z };
        CoordT end_coord_ = (radius_origin_coord_ + direction_coord_);
        if (rayCastPoint(radius_origin_coord_, end_coord_, casted_point_))
        {
          casted_points.push_back(casted_point_);
        }
      }
    }
    if (counter_ == 0)
    {
      valid_ratio = 0.0;
    }
    else
    {
      valid_ratio = casted_points.size() / (double)counter_;
    }
    return;
  }

  inline double ProbabilisticMap::getNearestOccupiedDistance(const Vector3D& point) {
    // 현재 좌표 주변의 일정 범위만 검색
    const double search_radius = 2.0 * _grid.resolution;
    const auto center_coord = grid().posToCoord(point);
    double min_dist = std::numeric_limits<double>::max();
    
    // 주변 셀만 검색
    for(int dx=-search_radius; dx<=search_radius; dx++) {
        for(int dy=-search_radius; dy<=search_radius; dy++) {
            for(int dz=-search_radius; dz<=search_radius; dz++) {
                CoordT neighbor = {
                    center_coord.x + dx,
                    center_coord.y + dy,
                    center_coord.z + dz
                };
                if(isOccupied(neighbor)) {
                    auto pos = grid().coordToPos(neighbor);
                    min_dist = std::min(min_dist, 
                        (point - Vector3D(pos.x, pos.y, pos.z)).norm());
                }
            }
        }
    }
    return min_dist;
  }

}  // namespace Bonxai