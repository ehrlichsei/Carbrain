#include "common/concurrent_queue.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <numeric>
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

using common::ConcurrentQueue;

TEST(ConcurrentQueueTests, initilizedEmptyAndNotFull) {
  ConcurrentQueue<int, 2> q;
  EXPECT_TRUE(q.empty());
  EXPECT_FALSE(q.full());
}

TEST(ConcurrentQueueTests, nonEmptyAfterPush) {
  ConcurrentQueue<int, 2> q;
  q.push(5);
  EXPECT_FALSE(q.empty());
  EXPECT_FALSE(q.full());
}

TEST(ConcurrentQueueTests, fullAfter2Push) {
  ConcurrentQueue<int, 2> q;
  q.push(5);
  q.push(5);
  EXPECT_FALSE(q.empty());
  EXPECT_TRUE(q.full());
}

TEST(ConcurrentQueueTests, emptyAfterPushPop) {
  ConcurrentQueue<int, 2> q;
  q.push(5);
  int x;
  EXPECT_TRUE(q.pop(&x));
  EXPECT_EQ(5, x);
  EXPECT_TRUE(q.empty());
  EXPECT_FALSE(q.full());
}

TEST(ConcurrentQueueTests, fifo) {
  ConcurrentQueue<int, 2> q;
  q.push(42);
  q.push(5);
  int x1, x2;
  EXPECT_TRUE(q.pop(&x1));
  EXPECT_EQ(42, x1);
  EXPECT_TRUE(q.pop(&x2));
  EXPECT_EQ(5, x2);
}

TEST(ConcurrentQueueTests, push_and_pop) {
  constexpr size_t n = 28;
  std::array<int, n> source;
  std::vector<int> destination;
  destination.reserve(n);

  std::iota(source.begin(), source.end(), 0);

  ConcurrentQueue<int, n> q;

  for (int s : source) {
    q.push(s);
  }

  while (!q.empty()) {
    int e;
    q.pop(&e);
    destination.push_back(e);
  }

  EXPECT_TRUE(std::equal(source.begin(), source.end(), destination.begin()));
}

TEST(ConcurrentQueueTests, mixed_push_and_pop) {
  constexpr size_t n = 28;
  std::array<int, n> source;
  std::vector<int> destination;
  destination.reserve(n);

  std::iota(source.begin(), source.end(), 0);

  ConcurrentQueue<int, n> q;

  for (size_t i = 0; i < source.size(); i++) {
    q.push(source.at(i));
    if (!(i % 3)) {
      int e;
      q.pop(&e);
      destination.push_back(e);
    }
  }

  while (!q.empty()) {
    int e;
    q.pop(&e);
    destination.push_back(e);
  }

  EXPECT_TRUE(std::equal(source.begin(), source.end(), destination.begin()));
}

TEST(ConcurrentQueueTests, fifo_ring) {
  ConcurrentQueue<int, 2> q;
  q.push(42);
  q.push(5);
  q.push(10);
  int x1, x2;
  EXPECT_TRUE(q.pop(&x1));
  EXPECT_EQ(5, x1);
  EXPECT_TRUE(q.pop(&x2));
  EXPECT_EQ(10, x2);
}

TEST(ConcurrentQueueTests, emtpyAfterExactNPops) {
  ConcurrentQueue<int, 2> q;
  q.push(1);
  q.push(2);
  q.push(3);
  int x;
  EXPECT_TRUE(q.pop(&x));
  EXPECT_TRUE(q.pop(&x));
  EXPECT_TRUE(q.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
