#include "frc/vision/calibration/capture_gate.h"

#include "gtest/gtest.h"

namespace frc::vision::testing {

using Verdict = CaptureGate::Verdict;

Eigen::Affine3d Pose(double x, double yaw = 0.0) {
  return Eigen::Translation3d(x, 0.0, 1.0) *
         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY());
}

TEST(CaptureGateTest, CapturesTheFirstFrame) {
  CaptureGate gate;
  EXPECT_EQ(gate.Judge(Pose(0.0)).verdict, Verdict::kCapture);
}

TEST(CaptureGateTest, WantsANewPoseThenStillness) {
  CaptureGate gate;
  gate.Captured(Pose(0.0));

  // 10 cm and 10 degrees: not far enough.
  CaptureGate::Decision decision = gate.Judge(Pose(0.1, 0.17));
  EXPECT_EQ(decision.verdict, Verdict::kMoveToNewPose);
  EXPECT_GT(decision.moved, 0.0);
  EXPECT_LT(decision.moved, 1.0);

  // 40 cm, but 30 cm since the last frame: moving.
  decision = gate.Judge(Pose(0.4));
  EXPECT_EQ(decision.verdict, Verdict::kHoldStill);
  EXPECT_GT(decision.motion, 1.0);

  // 5 mm more: still.
  decision = gate.Judge(Pose(0.405));
  EXPECT_EQ(decision.verdict, Verdict::kCapture);
  EXPECT_LT(decision.motion, 1.0);
  gate.Captured(Pose(0.405));

  // Right where the last capture was.
  EXPECT_EQ(gate.Judge(Pose(0.405)).verdict, Verdict::kMoveToNewPose);
}

TEST(CaptureGateTest, RotationAloneIsANewPose) {
  CaptureGate gate;
  gate.Captured(Pose(0.0));
  gate.Judge(Pose(0.0, 0.6));
  EXPECT_EQ(gate.Judge(Pose(0.0, 0.6)).verdict, Verdict::kCapture);
}

TEST(CaptureGateTest, UndoingEveryCaptureStartsOver) {
  CaptureGate gate;
  gate.Captured(Pose(0.0));
  EXPECT_EQ(gate.Judge(Pose(0.0)).verdict, Verdict::kMoveToNewPose);
  gate.set_last_capture(std::nullopt);
  EXPECT_EQ(gate.Judge(Pose(0.0)).verdict, Verdict::kCapture);
}

}  // namespace frc::vision::testing
