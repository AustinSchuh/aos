#include "frc/vision/util/charuco_board.h"

#include "gtest/gtest.h"

#include "aos/flatbuffers.h"

namespace frc::vision::testing {

TEST(CharucoBoardTest, PresetsAreValid) {
  ASSERT_FALSE(BoardPresets().empty());
  for (const NamedBoard &preset : BoardPresets()) {
    EXPECT_TRUE(ValidateBoardSpec(preset.spec).ok()) << preset.name;
  }
  // The default is the board the calibration tools default to.
  EXPECT_EQ(BoardPresets()[0].spec, BoardSpec{});
}

TEST(CharucoBoardTest, RefusesBadBoards) {
  BoardSpec spec;
  spec.squares_x = 2;
  EXPECT_FALSE(ValidateBoardSpec(spec).ok());

  spec = BoardSpec{};
  spec.marker_length = spec.square_length;
  EXPECT_FALSE(ValidateBoardSpec(spec).ok());

  spec = BoardSpec{};
  spec.square_length = 0.0;
  EXPECT_FALSE(ValidateBoardSpec(spec).ok());

  spec = BoardSpec{};
  spec.dictionary = "DICT_9X9_1";
  EXPECT_FALSE(ValidateBoardSpec(spec).ok());

  // 15x15 needs 112 markers, more than DICT_5X5_100 has.
  spec = BoardSpec{.squares_x = 15, .squares_y = 15};
  EXPECT_FALSE(ValidateBoardSpec(spec).ok());
  spec.dictionary = "DICT_5X5_250";
  EXPECT_TRUE(ValidateBoardSpec(spec).ok());
}

TEST(CharucoBoardTest, MakesTheBoard) {
  const BoardSpec spec;
  cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(spec);
  EXPECT_EQ(board->getChessboardSize(), cv::Size(14, 9));
  EXPECT_FLOAT_EQ(board->getSquareLength(), 0.04f);
  EXPECT_FLOAT_EQ(board->getMarkerLength(), 0.03f);
  // The inner corners: 13x8.
#if CV_VERSION_MINOR >= 9
  EXPECT_EQ(board->getChessboardCorners().size(), 13u * 8u);
#else
  EXPECT_EQ(board->chessboardCorners.size(), 13u * 8u);
#endif
}

TEST(CharucoBoardTest, RoundTripsThroughTheFlatbuffer) {
  const BoardSpec spec{12, 9, 0.06, 0.04666, "DICT_5X5_250"};
  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(PackBoardSpec(&fbb, spec));
  aos::FlatbufferDetachedBuffer<calibration::BoardSpec> buffer(fbb.Release());
  EXPECT_EQ(BoardSpecFromFlatbuffer(buffer.message()), spec);
}

}  // namespace frc::vision::testing
