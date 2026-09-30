/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2026, robot.com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of robot.com nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Pedro Gonzalez (pedro@robot.com)
 *          Johan Solarte (jsolarte@robot.com)
 *********************************************************************/

#ifndef SEMANTIC_SEGMENTATION_LAYER__SEGMENTATION_TILE_MAP_HPP_
#define SEMANTIC_SEGMENTATION_LAYER__SEGMENTATION_TILE_MAP_HPP_

#include <cmath>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "temporal_observation_queue.hpp"

/**
 * @brief Represents a 2D grid index with equality comparison. Supports negative indexes
 */
struct TileIndex
{
  int x, y;

  bool operator==(const TileIndex & other) const
  {
    return x == other.x && y == other.y;
  }
};

/**
 * @brief Represents the world coordinates of a tile.
 */
struct TileWorldXY
{
  double x, y;
};

/**
 * @brief Custom hash function for TileIndex to enable its use as a key in unordered_map.
 */
template<>
struct std::hash<TileIndex>
{
  size_t operator()(const TileIndex & coord) const
  {
    // Compute individual hash values for two integers
    // and combine them using bitwise XOR
    // and bit shifting:
    return std::hash<int>()(coord.x) ^ (std::hash<int>()(coord.y) << 1);
  }
};

/**
 * @brief Manages a map of tile observations, allowing for spatial and temporal querying.
 * Utilizes an unordered_map to efficiently index observations by tile and supports locking for thread safety.
 */
class SegmentationTileMap
{
private:
  std::unordered_map<TileIndex, TemporalObservationQueue> tile_map_;
  float resolution_;
  float decay_time_;
  int clear_frames_{0};
  // Tiles removed since the last takeRemovedTiles(); the layer resets their
  // costmap cells, which are otherwise only ever written for live tiles.
  std::vector<TileIndex> removed_tiles_;
  std::recursive_mutex lock_;

public:
  using SharedPtr = std::shared_ptr<SegmentationTileMap>;

  // Define iterator types
  using Iterator = typename std::unordered_map<TileIndex, TemporalObservationQueue>::iterator;
  using ConstIterator = typename std::unordered_map<TileIndex,
      TemporalObservationQueue>::const_iterator;

  SegmentationTileMap(float resolution, float decay_time, int clear_frames = 0)
  : resolution_(resolution), decay_time_(decay_time), clear_frames_(clear_frames)
  {
    tile_map_.reserve(1e4);
  }

  SegmentationTileMap() {}

  // Return iterator to the beginning of the tile_map_
  Iterator begin() {return tile_map_.begin();}
  ConstIterator begin() const {return tile_map_.begin();}

  // Return iterator to the end of the tile_map_
  Iterator end() {return tile_map_.end();}
  ConstIterator end() const {return tile_map_.end();}

  /**
   * @brief Locks the map for exclusive access.
   */
  inline void lock() {lock_.lock();}

  /**
   * @brief Unlocks the map.
   */
  inline void unlock() {lock_.unlock();}

  /**
   * @brief Returns the number of elements in the map.
   * @return The size of the map.
   */
  int size() {return tile_map_.size();}

  /**
   * @brief Returns the decay time for queue
   *
   * @return float
   */
  float getDecayTime() const {return decay_time_;}

  /**
   * @brief Converts world coordinates to a TileIndex.
   * @param x X coordinate in world space.
   * @param y Y coordinate in world space.
   * @return The corresponding TileIndex.
   */
  TileIndex worldToIndex(double x, double y) const
  {
    // Convert world coordinates to grid indices
    int ix = static_cast<int>(std::floor(x / resolution_));
    int iy = static_cast<int>(std::floor(y / resolution_));

    return TileIndex{ix, iy};
  }

  /**
   * @brief Converts a TileIndex to world coordinates.
   * @param idx The index to convert.
   * @return The world coordinates of the tile's center.
   */
  TileWorldXY indexToWorld(int x, int y) const
  {
    // Calculate the world coordinates of the center of the grid cell
    double x_world = (static_cast<double>(x) + 0.5) * resolution_;
    double y_world = (static_cast<double>(y) + 0.5) * resolution_;

    return TileWorldXY{x_world, y_world};
  }

  /**
   * @brief Adds an observation to the specified tile.
   * @param obs The observation to add.
   * @param idx The index of the tile.
   * @param dominant_priority Whether this class should take immediate dominance when observed.
   */
  void pushObservation(TileObservation & obs, TileIndex & idx, bool dominant_priority = false)
  {
    auto it = tile_map_.find(idx);
    if (it != tile_map_.end()) {
      // TileIndex exists, push the observation with dominance flag
      it->second.push(obs, dominant_priority);
    } else {
      // TileIndex does not exist, create a new TemporalObservationQueue with decay time
      TemporalObservationQueue & queue = tile_map_[idx];
      queue.setDecayTime(decay_time_);
      queue.setClearFrames(clear_frames_);
      queue.push(obs, dominant_priority);
    }
  }

  /**
   * @brief Records that an existing tile was observed as an unmarked class
   * (background or a class without a cost entry), removing the tile once the
   * contrary evidence reaches clear_frames.
   * @param idx The index of the tile.
   */
  void observeUnmarked(const TileIndex & idx)
  {
    auto it = tile_map_.find(idx);
    if (it == tile_map_.end()) {
      return;
    }
    it->second.observeUnmarked();
    if (it->second.empty()) {
      removed_tiles_.emplace_back(it->first);
      tile_map_.erase(it);
    }
  }

  /**
   * @brief Returns and forgets the tiles removed by decay or contrary evidence.
   */
  std::vector<TileIndex> takeRemovedTiles()
  {
    std::vector<TileIndex> removed;
    removed.swap(removed_tiles_);
    return removed;
  }

  /**
   * @brief Removes observations older than the decay time from all tiles.
   * @param current_time The current time for comparison.
   */
  void purgeOldObservations(double current_time)
  {
    std::vector<TileIndex> tiles_to_remove;
    for (auto & tile : tile_map_) {
      tile.second.purgeOld(current_time);
      if(tile.second.empty()) {
        tiles_to_remove.emplace_back(tile.first);
      }
    }
    if(tile_map_.size() > 0) {
      for (auto & tile : tiles_to_remove) {
        tile_map_.erase(tile);
        removed_tiles_.emplace_back(tile);
      }
    }
  }

  /**
   * @brief Removes tiles whose center lies outside the given world-frame box.
   *
   * Time decay alone does not bound the map: with a long tile_map_decay_time
   * every tile the robot has driven past stays alive, and every consumer
   * (cost update, purge, visualization) walks the whole map each cycle.
   * Tiles outside the costmap window can never be written to the costmap,
   * so dropping them caps the map at the window area.
   * @return The number of tiles removed.
   */
  size_t purgeOutsideBounds(double min_x, double min_y, double max_x, double max_y)
  {
    size_t removed = 0;
    for (auto it = tile_map_.begin(); it != tile_map_.end(); ) {
      TileWorldXY world = indexToWorld(it->first.x, it->first.y);
      if (world.x < min_x || world.x > max_x || world.y < min_y || world.y > max_y) {
        it = tile_map_.erase(it);
        ++removed;
      } else {
        ++it;
      }
    }
    return removed;
  }
};

#endif  // SEMANTIC_SEGMENTATION_LAYER__SEGMENTATION_TILE_MAP_HPP_
