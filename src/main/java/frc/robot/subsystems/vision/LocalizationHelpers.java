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
        double currentRotation = drivetrain.getPigeon2().getYaw().getValueAsDouble();//Get current from LL instead
        double xyStdDev;

        LimelightHelpers.SetRobotOrientation(LLName, currentRotation, angularVelocity, 0, 0, 0, 0);
        PoseEstimate MT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LLName);

        double linearVelocity = Math.hypot(
            drivetrain.getRobotRelativeSpeeds().vxMetersPerSecond,
            drivetrain.getRobotRelativeSpeeds().vyMetersPerSecond
        );

      if (MT2 == null) {
            // if (LLName.equals("limelight-a")) {
            //     Constants.visionValidAPub.set(false);
            // } else {
            //     Constants.visionValidBPub.set(false);
            // }
            return;
        }

        if (MT2.avgTagDist > 6.0 || MT2.avgTagArea < 0.08) {
            return;
        }

        if (Math.abs(angularVelocity) > 720) {
            doRejectUpdate = true;
        }

        boolean isInvalid = (MT2.tagCount == 0 || MT2.rawFiducials == null || MT2.rawFiducials.length < 1 || doRejectUpdate == true);

        updateDynamicCrop(LLName, isInvalid);

        if (!Constants.hasInitializedFromVision && !isInvalid) {
            Pose2d correctPose = new Pose2d(MT2.pose.getX(), MT2.pose.getY(), Rotation2d.fromDegrees(currentRotation));//Reset to LL rotaion instead
            drivetrain.resetPose(correctPose);
            Constants.hasInitializedFromVision = true;
        }


        //TODO: For removal
        // if (LLName.equals("limelight-a")) {
        //     Constants.visionValidAPub.set(!isInvalid);
        //     if (!isInvalid) {
        //         Constants.distanceToTagAPub.set(MT2.rawFiducials[0].distToRobot);
        //         Constants.visionPoseXAPub.set(MT2.pose.getX());
        //         Constants.visionPoseYAPub.set(MT2.pose.getY());
        //         Constants.visionPoseRotAPub.set(MT2.pose.getRotation().getDegrees());
        //     }
        // } else {
        //     Constants.visionValidBPub.set(!isInvalid);
        //     if (!isInvalid) {
        //         Constants.distanceToTagBPub.set(MT2.rawFiducials[0].distToRobot);
        //         Constants.visionPoseXBPub.set(MT2.pose.getX());
        //         Constants.visionPoseYBPub.set(MT2.pose.getY());
        //         Constants.visionPoseRotBPub.set(MT2.pose.getRotation().getDegrees());
        //     }
        // }

        if (!isInvalid) {

            xyStdDev = calculateStdDev(MT2, linearVelocity);


            drivetrain.addVisionMeasurement(MT2.pose, MT2.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        }
    }

    private static double calculateStdDev(PoseEstimate MT2, double linearVelocity) {
        double stdDev = 0.05;
        
        double distanceFactor = 1 + (MT2.avgTagDist * 0.3);
        stdDev *= (distanceFactor * distanceFactor);
        
        if (MT2.tagCount >= 2) {
            stdDev *= 0.5;
        }
        
        if (MT2.avgTagArea < 0.2) {
            double areaFactor = 0.2 / Math.max(MT2.avgTagArea, 0.05);
            stdDev *= Math.min(areaFactor, 3.0);
        }
        
        if (linearVelocity > 0.5) {
            double velocityFactor = 1 + (linearVelocity / 4.5);
            stdDev *= velocityFactor;
        }
        
        stdDev = Math.max(0.01, Math.min(stdDev, 5.0));
        
        return stdDev;
    }

    private static void updateDynamicCrop(String name, boolean isInvalid){
        if(isInvalid){
            LimelightHelpers.setCropWindow(name, -1.0, 1.0, -1.0, 1.0);
        } else {
            double centerX = LimelightHelpers.getTX(name) / 41.0;
            double centerY = LimelightHelpers.getTY(name) / 28.1;
            double cropRadius = 0.1;
            double xMin = centerX - cropRadius - 0.25;
            double xMax = centerX + cropRadius + 0.25;
            double yMin = centerY - cropRadius;
            double yMax = centerY + cropRadius;
            xMin = Math.max(-1.0, xMin);
            xMax = Math.min(1.0, xMax);
            yMin = Math.max(-1.0, yMin);
            yMax = Math.min(1.0, yMax);
            LimelightHelpers.setCropWindow(name, xMin, xMax, yMin, yMax);
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
