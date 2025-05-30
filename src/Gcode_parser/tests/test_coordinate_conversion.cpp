#include "coordinate_conversion.h"
#include <gtest/gtest.h>

using coordinate_conversion::latLonToLocal;

TEST(CoordinateConversionTest, ZeroDelta)
{
    auto enu = latLonToLocal(10.0, 20.0, 10.0, 20.0);
    EXPECT_NEAR(enu.east, 0.0, 1e-6);
    EXPECT_NEAR(enu.north, 0.0, 1e-6);
}

TEST(CoordinateConversionTest, SmallOffset)
{
    auto enu = latLonToLocal(10.001, 20.001, 10.0, 20.0);
    EXPECT_NEAR(enu.east, 109.6, 1.0);
    EXPECT_NEAR(enu.north, 111.3, 1.0);
}
