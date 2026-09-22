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
#include "semantic_segmentation_layer/temporal_observation_queue.hpp"

class QueueTestWrapper : public TemporalObservationQueue {
public:
  const std::deque<TileObservation> & getClassQueue(uint8_t class_id) const
  {
    auto it = class_queues_.find(class_id);

    if (it == class_queues_.end()) {
      std::stringstream msg;
      msg << "Get class queue: Class ID " << class_id << " doesn't exist!" << '\n';

      throw std::invalid_argument(msg.str());
    }

    return it->second;
  }

  std::size_t getClassQueuesSize() const
  {
    return class_queues_.size();
  }

  std::size_t getConfidenceSumsSize() const
  {
    return class_confidence_sums_.size();
  }
};

/**
 * Test push without dominant priority, dominance is based on queue size alone
 */
TEST(TestTemporalObservationQueue, test_push_non_dominant)
{
  QueueTestWrapper queue;
  queue.setDecayTime(1.0); // We won't be using this

  TileObservation class_1 {1, 1.0, 1.0};
  queue.push(class_1);

  // Some basic checks
  ASSERT_FALSE(queue.empty());
  EXPECT_EQ(queue.size(), 1);
  ASSERT_EQ(queue.getClassQueuesSize(), queue.getConfidenceSumsSize());

  // Class 1 should be the dominant class
  EXPECT_EQ(queue.getClassId(), 1);
  EXPECT_EQ(queue.getConfidenceSum(), 1.0);

  // No we add 2 observations of class 2 and see if that becomes dominant...
  TileObservation class_2 {2, 1.0, 2.0};
  queue.push(class_2);

  class_2.timestamp = 2.0;
  queue.push(class_2);

  EXPECT_EQ(queue.getClassId(), 2);
  EXPECT_EQ(queue.size(), 2);
  ASSERT_EQ(queue.getClassQueuesSize(), queue.getConfidenceSumsSize());

  // Queue for class 1 should be deleted
  EXPECT_THROW(queue.getClassQueue(1), std::invalid_argument);
}

TEST(TestTemporalObservationQueue, test_observation_cost_average_inputs)
{
  QueueTestWrapper queue;
  queue.setDecayTime(10.0);

  queue.push(TileObservation{3, 200.0f, 1.0, 110});
  queue.push(TileObservation{3, 200.0f, 2.0, 220});

  EXPECT_EQ(queue.size(), 2);
  EXPECT_FLOAT_EQ(queue.getCostSum(), 330.0f);

  queue.purgeOld(12.0);
  EXPECT_EQ(queue.size(), 1);
  EXPECT_FLOAT_EQ(queue.getCostSum(), 220.0f);
}

/**
 * Test push with dominant priority
 */
TEST(TestTemporalObservationQueue, test_push_dominant)
{
  QueueTestWrapper queue;
  queue.setDecayTime(1.0); // We won't be using this

  /**
   * Push two observations of class 1 and then a class 2 observation with
   * dominant priority to see if that supersedes
   */
  TileObservation class_1 {1, 1.0, 1.0};
  queue.push(class_1);

  class_1.timestamp = 2.0;
  queue.push(class_1);

  EXPECT_EQ(queue.getClassId(), 1);

  TileObservation class_2 {2, 1.0, 3.0};
  queue.push(class_2, true);

  EXPECT_EQ(queue.getClassId(), 2);
  EXPECT_THROW(queue.getClassQueue(1), std::invalid_argument);
}

/**
 * Test purge old observations normally
 */
TEST(TestTemporalObservationQueue, test_purge_old)
{
  QueueTestWrapper queue;
  queue.setDecayTime(1.0);

  TileObservation class_1 {1, 1.0, 1.0};
  queue.push(class_1);

  class_1.timestamp = 2.0;
  queue.push(class_1);

  // First observation deleted
  queue.purgeOld(3.0);
  EXPECT_NEAR(queue.getConfidenceSum(), 1.0, 1e-3);
  EXPECT_EQ(queue.size(), 1);
  ASSERT_EQ(queue.getClassQueuesSize(), queue.getConfidenceSumsSize());

  // Entire queue clears...
  queue.purgeOld(4.0);
  EXPECT_EQ(queue.size(), 0);
  EXPECT_EQ(queue.getClassId(), -1);
  ASSERT_EQ(queue.getClassQueuesSize(), queue.getConfidenceSumsSize());
}

/**
 * Test dominant class bookkeeping during purge
 */
TEST(TestTemporalObservationQueue, test_purge_dominant_bookkeeping)
{
  QueueTestWrapper queue;
  queue.setDecayTime(3.0);

  // Dominant class, will get old enough to be purged
  TileObservation class_1 {1, 1.0, 1.0};
  queue.push(class_1, true);

  class_1.timestamp = 2.0;
  queue.push(class_1, true);

  // Non dominant-classes
  TileObservation class_2 {2, 1.0, 3.0};
  queue.push(class_2);

  class_2.timestamp = 4.0;
  queue.push(class_2);

  TileObservation class_3 {3, 1.0, 5.0};
  queue.push(class_3);

  // Dominant class purged, class 2 might be the new dominant class
  queue.purgeOld(6.0);

  EXPECT_EQ(queue.getClassId(), 2);
  ASSERT_EQ(queue.getClassQueuesSize(), queue.getConfidenceSumsSize());
}
