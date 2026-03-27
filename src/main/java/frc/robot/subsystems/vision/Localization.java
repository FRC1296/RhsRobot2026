package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Localization - Vision-based pose estimation for swerve drivetrain.
 *
 * Architecture:
 * - IMU mode 1 (full external gyro trust) when disabled, mode 4 (blended) when
 * enabled
 * - MegaTag2 (gyro-constrained XY) used for ALL pose seeding and resets
 * - Heading is ALWAYS taken from the Pigeon (never from vision)
 * - Pigeon is set to 0 at boot and NEVER adjusted by this class
 * - 0 degrees = robot facing straight toward the opposite alliance wall
 * - MegaTag2 used for continuous XY localization (heading always
 * large-variance)
 * - MegaTag1 used only for rotation correction at low speed (XY + heading)
 * - Multi-camera fusion via inverse-variance weighting before integrating
 * - Dynamic std dev based on speed, tag count, target size, pose jump distance
 */
public class Localization {

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------

    /** 2026 field dimensions in meters */
    private static final double FIELD_LENGTH = 16.540988;
    private static final double FIELD_WIDTH = 8.069326;

    /** Max angular velocity (deg/s) before rejecting MT2 updates */
    private static final double MAX_ANGULAR_VELOCITY_DEG_S = 720.0;

    /** Max angular velocity (rad/s) before skipping MT1 rotation integration */
    private static final double MAX_OMEGA_FOR_ROTATION_RAD_S = 0.5;

    /** Rotation std dev sentinel — tells the estimator "don't fuse rotation" */
    private static final double LARGE_VARIANCE = 999999.0;

    /** Max time delta (seconds) between two estimates to consider them fuseable */
    private static final double MAX_TIME_DELTA_SECONDS = 0.1;

    /** Max tag distance (meters) to accept a pose */
    private static final double MAX_TAG_DIST = 6.0;

    /** Min tag area (%) to accept a pose */
    private static final double MIN_TAG_AREA = 0.025;

    /** Max tag distance for trusting MT2 as initial seed */
    private static final double MAX_SEED_DIST = 5.0;

    /** Max ambiguity for a tag to be accepted */
    private static final double MAX_AMBIGUITY = 0.9;

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /**
     * Call this every robot loop from robotPeriodic().
     * Pass all limelight names you want fused together.
     *
     * Example:
     * Localization.updatePose(drivetrain, "limelight-a", "limelight-b");
     */
    public static void updatePose(CommandSwerveDrivetrain drivetrain, String... llNames) {
        double angularVelocity = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        double currentYaw = drivetrain.getPigeon2().getYaw().getValueAsDouble();

        // Alliance-corrected heading for limelight orientation hint.
        // Note: we never modify the pigeon — this is only sent to the limelight
        // so MT2 knows which direction the robot is facing.
        double orientationDeg = currentYaw;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Blue) {
            orientationDeg += 180.0;
        }

        // IMU mode: 1 = full external trust (disabled/still), 4 = blended
        // (enabled/moving)
        int imuMode = DriverStation.isDisabled() ? 1 : 4;

        double linearVelocity = Math.hypot(
                drivetrain.getRobotRelativeSpeeds().vxMetersPerSecond,
                drivetrain.getRobotRelativeSpeeds().vyMetersPerSecond);

        for (String llName : llNames) {
            LimelightHelpers.SetIMUMode(llName, imuMode);
            LimelightHelpers.SetRobotOrientation(llName, orientationDeg, angularVelocity, 0, 0, 0, 0);
        }

        // --- Step 1: Initial seed using MT2 XY + Pigeon heading ---
        // MT2 needs the pigeon hint to compute XY (gyro-constrained).
        // The pigeon was set to 0 at boot and is never changed, so 0 = facing
        // straight toward the opposite alliance wall. We use that heading here.
        if (!Constants.hasInitializedFromVision) {
            for (String llName : llNames) {
                PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
                if (isValidMT2Seed(mt2)) {
                    // XY from MT2, heading from Pigeon (0 = straight to opposite wall)
                    drivetrain.resetPose(new Pose2d(
                            mt2.pose.getX(),
                            mt2.pose.getY(),
                            Rotation2d.fromDegrees(currentYaw)));
                    Constants.hasInitializedFromVision = true;
                    break; // one camera is enough to seed
                }
            }
            // Don't integrate measurements until we have a valid initial pose
            return;
        }

        // --- Step 2: Continuous XY updates from MT2 (all cameras fused) ---
        List<VisionEstimate> mt2Estimates = new ArrayList<>();
        for (String llName : llNames) {
            PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            VisionEstimate estimate = buildMT2Estimate(mt2, drivetrain, linearVelocity, angularVelocity);
            if (estimate != null) {
                mt2Estimates.add(estimate);
            }
        }
        integrateMultipleEstimates(drivetrain, mt2Estimates);

        // --- Step 3: MT1 rotation + XY correction when nearly still ---
        // MT1 solves full 6DOF from tag geometry and can give us a real heading.
        // Only trust it at low speed — rotation gets unreliable when spinning.
        if (linearVelocity <= 0.2) {
            List<VisionEstimate> mt1Estimates = new ArrayList<>();
            for (String llName : llNames) {
                PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);
                VisionEstimate estimate = buildMT1Estimate(mt1, drivetrain, linearVelocity, angularVelocity);
                if (estimate != null) {
                    mt1Estimates.add(estimate);
                }
            }
            integrateMultipleEstimates(drivetrain, mt1Estimates);
        }
    }

    /**
     * Resets drivetrain pose to the best available MT2 reading.
     * XY comes from whichever camera has the closest/most confident tag view.
     * Heading is always taken from the Pigeon — never from vision.
     *
     * Call this from a button binding or at auto init.
     */
    public static void resetToLimelightPose(CommandSwerveDrivetrain drivetrain, String... llNames) {
        double angularVelocity = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        double currentYaw = drivetrain.getPigeon2().getYaw().getValueAsDouble();

        double orientationDeg = currentYaw;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Blue) {
            orientationDeg += 180.0;
        }

        // Give all cameras the current orientation so MT2 XY is computed correctly
        for (String llName : llNames) {
            LimelightHelpers.SetIMUMode(llName, 1);
            LimelightHelpers.SetRobotOrientation(llName, orientationDeg, angularVelocity, 0, 0, 0, 0);
        }

        // Pick the MT2 estimate from the camera with the closest tag view
        PoseEstimate bestMT2 = null;
        double bestDist = Double.MAX_VALUE;

        for (String llName : llNames) {
            PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            if (isValidMT2Seed(mt2) && mt2.avgTagDist < bestDist) {
                bestDist = mt2.avgTagDist;
                bestMT2 = mt2;
            }
        }

        if (bestMT2 != null) {
            // XY from MT2, heading from Pigeon — pigeon is never touched here
            drivetrain.resetPose(new Pose2d(
                    bestMT2.pose.getX(),
                    bestMT2.pose.getY(),
                    Rotation2d.fromDegrees(currentYaw)));
            Constants.hasInitializedFromVision = true;
        }
    }

    /** Returns true if any of the given limelights has a target in view */
    public static boolean tagInView(String... llNames) {
        for (String llName : llNames) {
            if (LimelightHelpers.getTV(llName))
                return true;
        }
        return false;
    }

    // -------------------------------------------------------------------------
    // Estimate builders
    // -------------------------------------------------------------------------

    /**
     * Builds a VisionEstimate from a MegaTag2 pose.
     * Returns null if the estimate should be rejected.
     * Heading component always uses LARGE_VARIANCE — MT2 does not solve rotation.
     */
    private static VisionEstimate buildMT2Estimate(
            PoseEstimate mt2,
            CommandSwerveDrivetrain drivetrain,
            double linearVelocity,
            double angularVelocity) {

        if (mt2 == null || mt2.tagCount == 0 || mt2.rawFiducials == null)
            return null;

        // Rejection filters
        if (Math.abs(angularVelocity) > MAX_ANGULAR_VELOCITY_DEG_S)
            return null;
        if (mt2.avgTagDist > MAX_TAG_DIST)
            return null;
        if (mt2.avgTagArea < MIN_TAG_AREA)
            return null;
        if (hasHighAmbiguity(mt2.rawFiducials))
            return null;
        if (isPoseOutOfField(mt2.pose))
            return null;

        double mt2PoseDiff = drivetrain.getPose()
                .getTranslation()
                .getDistance(mt2.pose.getTranslation());

        boolean multiTags = mt2.tagCount >= 2;
        double targetSize = mt2.avgTagArea;

        // Tiered XY std dev — higher trust when closer / more tags / stationary
        double xyStds;
        if (linearVelocity <= 0.2 && targetSize > 4)
            xyStds = 0.1; // stationary + close
        else if (multiTags && targetSize > 2)
            xyStds = 0.1; // strong multi-tag
        else if (multiTags && targetSize > 0.2)
            xyStds = 0.25; // decent multi-tag
        else if (targetSize > 2 && (mt2PoseDiff < 0.5 || DriverStation.isDisabled()))
            xyStds = 0.5; // close single tag
        else if (targetSize > 1 && (mt2PoseDiff < 0.25 || DriverStation.isDisabled()))
            xyStds = 1.0; // proximity single tag
        else if (targetSize >= MIN_TAG_AREA)
            xyStds = 1.5; // distant/small tag
        else
            return null; // not confident enough

        // MT2 never provides reliable rotation — always suppress heading fusion
        return new VisionEstimate(mt2.pose, mt2.timestampSeconds,
                VecBuilder.fill(xyStds, xyStds, LARGE_VARIANCE),
                mt2.tagCount);
    }

    /**
     * Builds a VisionEstimate from a MegaTag1 pose (6DOF, includes rotation).
     * Returns null if the estimate should be rejected.
     * Only called during slow motion — MT1 rotation gets unreliable when spinning.
     */
    private static VisionEstimate buildMT1Estimate(
            PoseEstimate mt1,
            CommandSwerveDrivetrain drivetrain,
            double linearVelocity,
            double angularVelocity) {

        if (mt1 == null || mt1.tagCount == 0 || mt1.rawFiducials == null)
            return null;

        // Rejection filters
        if (mt1.avgTagDist > MAX_TAG_DIST)
            return null;
        if (mt1.avgTagArea < MIN_TAG_AREA)
            return null;
        if (hasHighAmbiguity(mt1.rawFiducials))
            return null;
        if (isPoseOutOfField(mt1.pose))
            return null;
        if (isTilted(mt1))
            return null;

        double mt1PoseDiff = drivetrain.getPose().getTranslation().getDistance(mt1.pose.getTranslation());
        boolean multiTags = mt1.tagCount >= 2;
        double targetSize = mt1.avgTagArea;
        double highestAmbiguity = getHighestAmbiguity(mt1.rawFiducials);

        // Tiered XY + heading std dev
        double xyStds;
        double degStds;

        if (linearVelocity <= 0.2 && targetSize > 4) {
            xyStds = 0.1;
            degStds = 0.1;
        } else if (multiTags && targetSize > 2) {
            xyStds = 0.1;
            degStds = 0.1;
        } else if (multiTags && targetSize > 0.2) {
            xyStds = 0.25;
            degStds = 8.0;
        } else if (targetSize > 2 && mt1PoseDiff < 0.5) {
            xyStds = 0.5;
            degStds = LARGE_VARIANCE;
        } else if (targetSize > 1 && mt1PoseDiff < 0.25) {
            xyStds = 1.0;
            degStds = LARGE_VARIANCE;
        } else if (highestAmbiguity < 0.25 && targetSize >= MIN_TAG_AREA) {
            xyStds = 1.5;
            degStds = LARGE_VARIANCE;
        } else
            return null;

        // Weaken rotation trust when ambiguity is elevated
        if (highestAmbiguity > 0.5) {
            degStds = Math.max(degStds, 50.0);
        }

        // Suppress rotation fusion when spinning
        if (Math.abs(Math.toRadians(angularVelocity)) >= MAX_OMEGA_FOR_ROTATION_RAD_S) {
            degStds = Math.max(degStds, 75.0);
        }

        return new VisionEstimate(mt1.pose, mt1.timestampSeconds,
                VecBuilder.fill(xyStds, xyStds, degStds),
                mt1.tagCount);
    }

    // -------------------------------------------------------------------------
    // Multi-camera fusion (inverse-variance weighting)
    // -------------------------------------------------------------------------

    /**
     * Fuses a list of estimates from different cameras before integrating.
     * Estimates close in time are combined via inverse-variance weighting so the
     * drivetrain estimator receives one high-quality measurement per loop instead
     * of several conflicting ones.
     */
    private static void integrateMultipleEstimates(
            CommandSwerveDrivetrain drivetrain,
            List<VisionEstimate> estimates) {

        if (estimates.isEmpty())
            return;

        estimates.sort(Comparator.comparingDouble(e -> e.timestamp)); // oldest first

        VisionEstimate group = estimates.get(0);
        for (int i = 1; i < estimates.size(); i++) {
            VisionEstimate next = estimates.get(i);
            if (Math.abs(next.timestamp - group.timestamp) < MAX_TIME_DELTA_SECONDS) {
                group = fuseEstimates(drivetrain, group, next);
            } else {
                integrateSingle(drivetrain, group);
                group = next;
            }
        }
        integrateSingle(drivetrain, group);
    }

    private static void integrateSingle(CommandSwerveDrivetrain drivetrain, VisionEstimate estimate) {
        drivetrain.addVisionMeasurement(estimate.pose, estimate.timestamp, estimate.stdDevs);
    }

    /**
     * Fuses two camera estimates into one via inverse-variance weighting.
     * Projects the older estimate forward to the newer timestamp using odometry
     * delta.
     */
    private static VisionEstimate fuseEstimates(
            CommandSwerveDrivetrain drivetrain,
            VisionEstimate a,
            VisionEstimate b) {

        // Guarantee b is the newer one
        if (b.timestamp < a.timestamp) {
            VisionEstimate tmp = a;
            a = b;
            b = tmp;
        }

        Pose2d aPoseAtA = drivetrain.getPoseAtTimestamp(a.timestamp);
        Pose2d aPoseAtB = drivetrain.getPoseAtTimestamp(b.timestamp);
        Pose2d poseA = a.pose.transformBy(aPoseAtB.minus(aPoseAtA));
        Pose2d poseB = b.pose;

        double varAx = sq(a.stdDevs.get(0, 0));
        double varAy = sq(a.stdDevs.get(1, 0));
        double varAth = sq(a.stdDevs.get(2, 0));
        double varBx = sq(b.stdDevs.get(0, 0));
        double varBy = sq(b.stdDevs.get(1, 0));
        double varBth = sq(b.stdDevs.get(2, 0));

        double wAx = 1.0 / varAx, wAy = 1.0 / varAy;
        double wBx = 1.0 / varBx, wBy = 1.0 / varBy;

        double fusedX = (poseA.getX() * wAx + poseB.getX() * wBx) / (wAx + wBx);
        double fusedY = (poseA.getY() * wAy + poseB.getY() * wBy) / (wAy + wBy);

        Rotation2d fusedHeading;
        if (varAth < LARGE_VARIANCE && varBth < LARGE_VARIANCE) {
            fusedHeading = new Rotation2d(
                    poseA.getRotation().getCos() / varAth + poseB.getRotation().getCos() / varBth,
                    poseA.getRotation().getSin() / varAth + poseB.getRotation().getSin() / varBth);
        } else {
            fusedHeading = (varAth < LARGE_VARIANCE) ? poseA.getRotation() : poseB.getRotation();
        }

        Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(
                Math.sqrt(1.0 / (wAx + wBx)),
                Math.sqrt(1.0 / (wAy + wBy)),
                Math.sqrt(1.0 / (1.0 / varAth + 1.0 / varBth)));

        return new VisionEstimate(new Pose2d(fusedX, fusedY, fusedHeading),
                b.timestamp, fusedStdDevs, a.numTags + b.numTags);
    }

    private static double sq(double x) {
        return x * x;
    }

    // -------------------------------------------------------------------------
    // Rejection helpers
    // -------------------------------------------------------------------------

    private static boolean isPoseOutOfField(Pose2d pose) {
        return pose.getX() < 0 || pose.getX() > FIELD_LENGTH
                || pose.getY() < 0 || pose.getY() > FIELD_WIDTH;
    }

    private static boolean hasHighAmbiguity(RawFiducial[] fiducials) {
        if (fiducials == null)
            return false;
        for (RawFiducial f : fiducials) {
            if (f.ambiguity > MAX_AMBIGUITY)
                return true;
        }
        return false;
    }

    private static double getHighestAmbiguity(RawFiducial[] fiducials) {
        double max = 0;
        if (fiducials == null)
            return max;
        for (RawFiducial f : fiducials) {
            if (f.ambiguity > max)
                max = f.ambiguity;
        }
        return max;
    }

    /**
     * Returns true if the MT1 3D pose reports significant roll or pitch.
     * Currently a no-op safe fallback. To enable, fetch getBotPose3d_wpiBlue
     * and check rotation.getX()/getY() > MAX_TILT_DEG (5°).
     */
    private static boolean isTilted(PoseEstimate mt1) {
        return false;
    }

    /**
     * Validates that an MT2 estimate is good enough to use as an initial seed
     * or pose reset. Requires: non-null, has tags, close enough, low ambiguity,
     * inside field boundaries.
     */
    private static boolean isValidMT2Seed(PoseEstimate mt2) {
        if (mt2 == null)
            return false;
        if (mt2.tagCount == 0)
            return false;
        if (mt2.rawFiducials == null)
            return false;
        if (mt2.rawFiducials.length == 0)
            return false;
        if (mt2.avgTagDist > MAX_SEED_DIST)
            return false;
        if (mt2.avgTagArea < MIN_TAG_AREA)
            return false;
        if (hasHighAmbiguity(mt2.rawFiducials))
            return false;
        if (isPoseOutOfField(mt2.pose))
            return false;
        return true;
    }

    // -------------------------------------------------------------------------
    // Internal data class
    // -------------------------------------------------------------------------

    private static class VisionEstimate {
        final Pose2d pose;
        final double timestamp;
        final Matrix<N3, N1> stdDevs;
        final int numTags;

        VisionEstimate(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs, int numTags) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.stdDevs = stdDevs;
            this.numTags = numTags;
        }
    }
}