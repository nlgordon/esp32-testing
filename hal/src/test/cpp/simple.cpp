#include "gtest/gtest.h"
#include <driver/gpio.h>

namespace {
	class SimpleTest : public ::testing::Test { };

	TEST_F(SimpleTest, AlwaysSuccessful) {
		EXPECT_EQ(1, 1);
	}
}
