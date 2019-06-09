#include "common/path_conversion.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <boost/range/algorithm/equal.hpp>
THIRD_PARTY_HEADERS_END

#include "common/types.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(PathConversion, consistencyVector3d) {
  const common::Vector3dVector ps = {{3, 1, 5}, {-3.5, 42, -1}, {0, 1, 3}};
  common::Vector3dVector os;
  common::fromMsg(common::createPathMsg(ps, {}, ""), os);
  EXPECT_TRUE(boost::equal(ps, os));
}


TEST(PathConversion, consistencyVector2d) {
  const common::Vector2dVector ps = {{3, 5}, {42, -1}, {0, 1}};
  common::Vector2dVector os;
  common::fromMsg(common::createPathMsg(ps, {}, ""), os);
  EXPECT_TRUE(boost::equal(ps, os));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
