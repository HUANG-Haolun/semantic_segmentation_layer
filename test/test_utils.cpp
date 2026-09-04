// Copyright (c) 2026, Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include "gtest/gtest.h"
#include "semantic_segmentation_layer/utils.hpp"

/**
 * @brief Send an empty tile map and expect pointcloud header and fields to still set correctly.
 *
 */
TEST(TestVisualizeTileMap, test_empty_map)
{
  SegmentationTileMap empty_tile_map;

  std_msgs::msg::Header test_header;
  test_header.frame_id = "camera";
  test_header.stamp = rclcpp::Time(1, 0);

  auto pc = visualizeTemporalTileMap(empty_tile_map, test_header.frame_id, test_header.stamp);

  EXPECT_TRUE(pc->data.empty());
  EXPECT_EQ(pc->header, test_header);

  EXPECT_EQ(pc->fields.size(), 6);

  EXPECT_EQ(pc->fields.at(0).name, "x");
  EXPECT_EQ(pc->fields.at(0).datatype, sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_EQ(pc->fields.at(1).name, "y");
  EXPECT_EQ(pc->fields.at(1).datatype, sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_EQ(pc->fields.at(2).name, "z");
  EXPECT_EQ(pc->fields.at(2).datatype, sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_EQ(pc->fields.at(3).name, "confidence");
  EXPECT_EQ(pc->fields.at(3).datatype, sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_EQ(pc->fields.at(4).name, "confidence_avg");
  EXPECT_EQ(pc->fields.at(4).datatype, sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_EQ(pc->fields.at(5).name, "class");
  EXPECT_EQ(pc->fields.at(5).datatype, sensor_msgs::msg::PointField::UINT8);
}

/**
 * @brief Three observations from the same class, same location to see if points stack correctly
 *
 */
TEST(TestVisualizeTileMap, test_same_multiple_tiles)
{
  SegmentationTileMap tile_map;

  TileIndex index_1_1;
  index_1_1.x = 1;
  index_1_1.y = 1;

  TileObservation observation;
  observation.class_id = 1;
  observation.confidence = 1.0;

  for (int t = 0; t <= 3; ++t) {
    auto time = static_cast<double>(t);
    observation.timestamp = time;
    tile_map.pushObservation(observation, index_1_1);
  }

  std_msgs::msg::Header map_header;
  map_header.frame_id = "camera";
  map_header.stamp = rclcpp::Time(3, 0);

  auto pc = visualizeTemporalTileMap(tile_map, map_header.frame_id, map_header.stamp);

  /**
   * Check if tile information got correctly encoded into points
   *
   * Also check if the tiles on the same index stacked correctly as points on top of each
   * other separated by z-height
   *
   * Points from the point cloud are extracted to a vector so that they can be sorted
   * in ascending order to compare with expected z-height difference between points
   */
  std::vector<PointData> points;

  sensor_msgs::PointCloud2Iterator<float> i_x(*pc, "x");
  sensor_msgs::PointCloud2Iterator<float> i_y(*pc, "y");
  sensor_msgs::PointCloud2Iterator<float> i_z(*pc, "z");
  sensor_msgs::PointCloud2Iterator<float> i_conf(*pc, "confidence");
  sensor_msgs::PointCloud2Iterator<float> i_conf_avg(*pc, "confidence_avg");
  sensor_msgs::PointCloud2Iterator<uint8_t> i_class(*pc, "class");

  for (; i_x != i_x.end(); ++i_x, ++i_y, ++i_z, ++i_conf, ++i_conf_avg, ++i_class) {
    PointData point;
    point.class_id = *i_class;
    point.confidence = *i_conf;
    point.confidence_avg = *i_conf_avg;
    point.x = *i_x;
    point.y = *i_y;
    point.z = *i_z;

    points.push_back(point);
  }

  // Rearrange points by z height
  std::sort(points.begin(), points.end(),
    [](const PointData & lhs, const PointData & rhs){return lhs.z < rhs.z;});

  PointData last_point;
  last_point.class_id = 0;

  for (const auto & point : points) {
    EXPECT_EQ(point.class_id, 1);
    EXPECT_NEAR(point.confidence, 1.0, 1e-3);

    TileIndex index = tile_map.worldToIndex(point.x, point.y);
    // Braced init: parenthesized aggregate init is C++20; Jazzy nav2_package() sets C++17.
    EXPECT_EQ(index, (TileIndex{1, 1}));

    if (last_point.class_id == 0) {
      last_point = point;
      continue;
    }
    // Exact z-value in code
    EXPECT_NEAR(point.z - last_point.z, 0.02, 1e-2);

    last_point = point;
  }

  // Average of 3 observations with 1.0 confidence.
  EXPECT_NEAR(last_point.confidence_avg, 1.0, 1e-4);
}
