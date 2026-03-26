package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class LocalizationHelpers {

    public static void updatePose(CommandSwerveDrivetrain drivetrain, String llName) {
        double angularVelocity = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        double currentRotation = drivetrain.getPigeon2().getYaw().getValueAsDouble();

        if (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Blue) {
            currentRotation += 180.0;
        }

        LimelightHelpers.SetRobotOrientation(llName, currentRotation, angularVelocity, 0, 0, 0, 0);
        PoseEstimate MT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);

        double linearVelocity = Math.hypot(
                drivetrain.getRobotRelativeSpeeds().vxMetersPerSecond,
                drivetrain.getRobotRelativeSpeeds().vyMetersPerSecond);

        if (MT2 != null && MT2.avgTagDist <= 6.0 && MT2.avgTagArea >= 0.08) {
            boolean isValid = false;
            if (Math.abs(angularVelocity) <= 720 && isValidTarget(MT2)) {
                isValid = true;
            }

            if (!Constants.hasInitializedFromVision && isValid) {
                
                Pose2d correctPose = new Pose2d(
                        MT2.pose.getX(),
                        MT2.pose.getY(),
                        Rotation2d.fromDegrees(currentRotation));
                    drivetrain.resetPose(correctPose);
                    Constants.hasInitializedFromVision = true;
            } else if (Constants.hasInitializedFromVision && isValid) {
                double xyStdDev = calculateStdDev(MT2, linearVelocity);
                drivetrain.addVisionMeasurement(MT2.pose, MT2.timestampSeconds,
                        VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
            }
        }
    }

    

    private static boolean isValidTarget(PoseEstimate target) {
        boolean retVal = false;

        if (target.tagCount > 0 && target.rawFiducials != null && target.rawFiducials.length >= 1) {
            retVal = true;
        }

        return retVal;
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

    private static void updateDynamicCrop(String name, boolean isInvalid) {
        if (isInvalid) {
            LimelightHelpers.setCropWindow(name, -1.0, 1.0, -1.0, 1.0);
        } else {
            double centerX = LimelightHelpers.getTX(name) / 41.0;
            double centerY = LimelightHelpers.getTY(name) / 28.1;
            double cropRadius = 0.25;
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

        if (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Blue) {
            currentRotation += 180.0;
        }

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

    public static boolean tagInVison(String LLName) {
        return LimelightHelpers.getTV(LLName);
    }

}
