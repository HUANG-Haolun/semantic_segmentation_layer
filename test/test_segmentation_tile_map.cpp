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
#include "semantic_segmentation_layer/segmentation_tile_map.hpp"

TEST(TestSegmentationTileMap, test_world_index_conversions)
{
    SegmentationTileMap tile_map {0.25, 1.0};

    auto index = tile_map.worldToIndex(25, 25);
    EXPECT_EQ(index.x, 100);
    EXPECT_EQ(index.y, 100);

    auto coords = tile_map.indexToWorld(4, 4);
    EXPECT_NEAR(coords.x, 1.125, 1e-3);
    EXPECT_NEAR(coords.y, 1.125, 1e-3);
}

/**
 * Make sure that push function correctly sets the decay time for observation queues
 * since they don't set by themselves/by default which can lead to undefined behavior
 */
TEST(TestSegmentationTileMap, test_push_set_decay_time)
 {
    SegmentationTileMap tile_map {0.25, 1.0};

    TileObservation observation {1, 1.0, 1.0};
    TileIndex index {1, 1};

    tile_map.pushObservation(observation, index);

    /**
     * Since we cannot directly check the variable for decay time set within queue,
     * let's just test the decay behavior.
     */
    tile_map.purgeOldObservations(3.0);
    EXPECT_EQ(tile_map.size(), 0);
}

/**
 * Long decay times must not let tiles the robot has left behind accumulate:
 * tiles outside the costmap window are dropped regardless of their age.
 */
TEST(TestSegmentationTileMap, test_purge_outside_bounds)
{
    SegmentationTileMap tile_map {0.5, 3600.0};

    TileObservation observation {1, 1.0, 1.0};
    for (int x = -20; x < 20; ++x) {
        for (int y = -20; y < 20; ++y) {
            TileIndex index {x, y};
            tile_map.pushObservation(observation, index);
        }
    }
    ASSERT_EQ(tile_map.size(), 1600);

    // Window [-2.5, 2.5] x [-2.5, 2.5] at 0.5 m keeps tile centers -2.25..2.25.
    EXPECT_EQ(tile_map.purgeOutsideBounds(-2.5, -2.5, 2.5, 2.5), 1500u);
    EXPECT_EQ(tile_map.size(), 100);
    for (const auto & tile : tile_map) {
        TileWorldXY world = tile_map.indexToWorld(tile.first.x, tile.first.y);
        EXPECT_GE(world.x, -2.5);
        EXPECT_LE(world.x, 2.5);
        EXPECT_GE(world.y, -2.5);
        EXPECT_LE(world.y, 2.5);
    }

    // Nothing else is removed while the window stays put, and time decay
    // still applies to the tiles that remain.
    EXPECT_EQ(tile_map.purgeOutsideBounds(-2.5, -2.5, 2.5, 2.5), 0u);
    tile_map.purgeOldObservations(10.0);
    EXPECT_EQ(tile_map.size(), 100);
}

/**
 * Tiles removed by contrary evidence or by decay are reported once so the
 * layer can reset their costmap cells.
 */
TEST(TestSegmentationTileMap, test_removed_tiles_are_reported)
{
    SegmentationTileMap tile_map {0.5, 5.0, 2};

    TileObservation grass {2, 200.0f, 1.0};
    TileIndex seen_as_background {0, 0};
    TileIndex decays {3, 3};
    tile_map.pushObservation(grass, seen_as_background);
    tile_map.pushObservation(grass, decays);

    tile_map.observeUnmarked(seen_as_background);
    EXPECT_EQ(tile_map.size(), 2);
    tile_map.observeUnmarked(seen_as_background);
    EXPECT_EQ(tile_map.size(), 1);

    // Unknown tiles are ignored rather than created.
    tile_map.observeUnmarked(TileIndex{9, 9});
    EXPECT_EQ(tile_map.size(), 1);

    tile_map.purgeOldObservations(10.0);
    EXPECT_EQ(tile_map.size(), 0);

    auto removed = tile_map.takeRemovedTiles();
    ASSERT_EQ(removed.size(), 2u);
    EXPECT_EQ(removed[0], seen_as_background);
    EXPECT_EQ(removed[1], decays);
    EXPECT_TRUE(tile_map.takeRemovedTiles().empty());
}
