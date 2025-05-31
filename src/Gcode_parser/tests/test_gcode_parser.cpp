#include "gcode_parser.h"
#include <gtest/gtest.h>
#include <filesystem>

TEST(GCodeParserTest, ParsesSampleFile) {
    std::filesystem::path data_dir(TEST_DATA_DIR);
    auto file = data_dir / "sample.gcode";

    GCodeParser parser;
    auto commands = parser.parseFile(file.string());

    ASSERT_EQ(commands.size(), 3u);

    EXPECT_EQ(commands[0].code, 0);
    EXPECT_DOUBLE_EQ(commands[0].latitude, 10.0);
    EXPECT_DOUBLE_EQ(commands[0].longitude, 20.0);
    EXPECT_DOUBLE_EQ(commands[0].altitude, 3.0);
    EXPECT_DOUBLE_EQ(commands[0].feed_rate, 1000.0);

    EXPECT_EQ(commands[1].code, 1);
    EXPECT_DOUBLE_EQ(commands[1].latitude, 15.0);
    EXPECT_DOUBLE_EQ(commands[1].longitude, 25.0);
    EXPECT_FALSE(commands[1].has_alt);

    EXPECT_EQ(commands[2].code, 1);
    EXPECT_DOUBLE_EQ(commands[2].altitude, 5.0);
}

TEST(GCodeParserTest, HandlesComments) {
    std::filesystem::path data_dir(TEST_DATA_DIR);
    auto file = data_dir / "comments.gcode";

    GCodeParser parser;
    auto commands = parser.parseFile(file.string());

    ASSERT_EQ(commands.size(), 3u);
    EXPECT_EQ(commands[0].code, 0);
    EXPECT_EQ(commands[1].code, 1);
    EXPECT_EQ(commands[2].code, 1);
}

TEST(GCodeParserTest, ParsesPlowCommands) {
    std::filesystem::path data_dir(TEST_DATA_DIR);
    auto file = data_dir / "plow.gcode";

    GCodeParser parser;
    auto commands = parser.parseFile(file.string());

    ASSERT_EQ(commands.size(), 4u);
    EXPECT_EQ(commands[0].type, GCodeCommand::DropPlow);
    EXPECT_EQ(commands[1].type, GCodeCommand::Move);
    EXPECT_EQ(commands[2].type, GCodeCommand::LiftPlow);
    EXPECT_EQ(commands[3].type, GCodeCommand::Move);
}

