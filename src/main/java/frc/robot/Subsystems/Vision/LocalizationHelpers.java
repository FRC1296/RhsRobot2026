package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class LocalizationHelpers {

    public static void updateFieldPosition(CommandSwerveDrivetrain drivetrain, String LLName) {
        boolean doRejectUpdate = false;
        double angularVelocity = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        double currentRotation = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        double xyStdDev = 0.1;

        LimelightHelpers.SetRobotOrientation(LLName, currentRotation, angularVelocity, 0, 0, 0, 0);
        PoseEstimate MT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LLName);

        if (MT2 == null) {
            if (LLName.equals("limelight-front")) {
                Constants.visionValidFrontPub.set(false);
            } else {
                Constants.visionValidRearPub.set(false);
            }
            return;
        }

        if (Math.abs(angularVelocity) > 360) {
            doRejectUpdate = true;
        }

        boolean isInvalid = (MT2.tagCount == 0 || MT2.rawFiducials == null || MT2.rawFiducials.length < 1 || doRejectUpdate == true);

        if (!Constants.hasInitializedFromVision && !isInvalid) {
            Pose2d correctPose = new Pose2d(MT2.pose.getX(), MT2.pose.getY(), Rotation2d.fromDegrees(currentRotation));
            drivetrain.resetPose(correctPose);
            Constants.hasInitializedFromVision = true;
        }

        if (LLName.equals("limelight-front")) {
            Constants.visionValidFrontPub.set(!isInvalid);
            if (!isInvalid) {
                Constants.distanceToTagFrontPub.set(MT2.rawFiducials[0].distToRobot);
                Constants.visionPoseXFrontPub.set(MT2.pose.getX());
                Constants.visionPoseYFrontPub.set(MT2.pose.getY());
                Constants.visionPoseRotFrontPub.set(MT2.pose.getRotation().getDegrees());
            }
        } else {
            Constants.visionValidRearPub.set(!isInvalid);
            if (!isInvalid) {
                Constants.distanceToTagRearPub.set(MT2.rawFiducials[0].distToRobot);
                Constants.visionPoseXRearPub.set(MT2.pose.getX());
                Constants.visionPoseYRearPub.set(MT2.pose.getY());
                Constants.visionPoseRotRearPub.set(MT2.pose.getRotation().getDegrees());
            }
        }

        if (!isInvalid) {

            double avgDistance = MT2.avgTagDist;
            double avgArea = MT2.avgTagArea;

            xyStdDev = xyStdDev * (1 + (avgDistance / 4.0));

            if (MT2.tagCount >= 2) {
                xyStdDev *= 0.5;
            }

            if (avgArea < 0.15) {
                xyStdDev *= 1.5;
            }

        }

        if (!isInvalid) {
            drivetrain.addVisionMeasurement(MT2.pose, MT2.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        }
    }

    public static void resetToLimelightPose(CommandSwerveDrivetrain drivetrain, String LLName1, String LLName2) {
        double angularVelocity = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        double currentRotation = drivetrain.getPigeon2().getYaw().getValueAsDouble();

        LimelightHelpers.SetRobotOrientation(LLName1, currentRotation, angularVelocity, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(LLName2, currentRotation, angularVelocity, 0, 0, 0, 0);

        PoseEstimate OneMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LLName1);
        PoseEstimate TwoMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LLName2);

        boolean isInvalid1 = (OneMT2 == null || OneMT2.tagCount == 0 || OneMT2.rawFiducials == null || OneMT2.rawFiducials.length == 0);
        boolean isInvalid2 = (TwoMT2 == null || TwoMT2.tagCount == 0 || TwoMT2.rawFiducials == null || TwoMT2.rawFiducials.length == 0);

        if (!isInvalid1 && !isInvalid2) {
            double distToTag1 = OneMT2.rawFiducials[0].distToRobot;
            double distToTag2 = TwoMT2.rawFiducials[0].distToRobot;
            if (distToTag1 < distToTag2) {
                drivetrain.resetPose(new Pose2d(OneMT2.pose.getX(), OneMT2.pose.getY(), Rotation2d.fromDegrees(currentRotation)));
            } else {
                drivetrain.resetPose(new Pose2d(TwoMT2.pose.getX(), TwoMT2.pose.getY(), Rotation2d.fromDegrees(currentRotation)));
            }
        } else if (!isInvalid1 && isInvalid2) {
            drivetrain.resetPose(new Pose2d(OneMT2.pose.getX(), OneMT2.pose.getY(), Rotation2d.fromDegrees(currentRotation)));
        } else if (!isInvalid2 && isInvalid1) {
            drivetrain.resetPose(new Pose2d(TwoMT2.pose.getX(), TwoMT2.pose.getY(), Rotation2d.fromDegrees(currentRotation)));
        }
    }

}
