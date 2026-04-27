package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class ShotCalculator extends SubsystemBase {

    // -------------------------------------------------------------------------
    // Hub tables (original, unchanged)
    // -------------------------------------------------------------------------
    protected InterpolatingDoubleTreeMap shooterSpeedTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
    protected double shooterInterpSpeedAdjustment = 0.0;
    protected double ToFInterpAdjustment = 0;

    // -------------------------------------------------------------------------
    // Feed tables (new)
    // Hood is always commanded to FEED_HOOD_MAX for feed shots - no hood table needed.
    // -------------------------------------------------------------------------
    protected InterpolatingDoubleTreeMap feedShooterSpeedTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap feedTimeOfFlightTable = new InterpolatingDoubleTreeMap();
    public static final double FEED_HOOD_MAX = 0.625;

    // -------------------------------------------------------------------------
    // Dependencies (moved here from ShooterSubsystem)
    // -------------------------------------------------------------------------
    private CommandSwerveDrivetrain drivetrain;
    private Transform2d shooterOffset;
    private DoubleSubscriber robotVelocitySubscriber;

    public ShotCalculator(String name, CommandSwerveDrivetrain drivetrain, Transform2d shooterOffset) {
        super(name);
        this.drivetrain = drivetrain;
        this.shooterOffset = shooterOffset;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable robotTable = inst.getTable(Constants.NETWORK_TABLE);
        NetworkTable driveTable = robotTable.getSubTable(Constants.NT_DRIVE);
        robotVelocitySubscriber = driveTable.getDoubleTopic(Constants.NT_DRIVE_VELOCITY).subscribe(0.0);

        initializeTables();
    }

    protected CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    protected Transform2d getShooterOffset() {
        return shooterOffset;
    }

    public void initializeTables() {
        // ---- Hub shooter speed table (original, unchanged) ----
        shooterSpeedTable.put(1.25, 30.0);
        shooterSpeedTable.put(1.5, 33.25);
        shooterSpeedTable.put(2.0, 34.0);
        shooterSpeedTable.put(2.5, 35.5);
        shooterSpeedTable.put(3.0, 37.0);
        shooterSpeedTable.put(3.5, 38.75);
        shooterSpeedTable.put(4.0, 40.5);
        shooterSpeedTable.put(4.5, 43.0);
        shooterSpeedTable.put(5.0, 44.5);
        shooterSpeedTable.put(5.40, 46.5);
        shooterSpeedTable.put(6.0, 47.5);
        shooterSpeedTable.put(6.5, 50.0);

        // ---- Hub hood table (original, unchanged) ----
        hoodTable.put(1.5, 0.05);
        hoodTable.put(5.5, 0.625);

        //Distance to hub -> Time of Flight sec (original, unchanged)
        timeOfFlightTable.put(1.25, 0.98);
        timeOfFlightTable.put(1.5, 1.15);
        timeOfFlightTable.put(2.0, 1.16);
        timeOfFlightTable.put(2.5, 1.18);
        timeOfFlightTable.put(3.0, 1.20);
        timeOfFlightTable.put(3.5, 1.06);
        timeOfFlightTable.put(4.0, 1.10);
        timeOfFlightTable.put(4.5, 1.18);
        timeOfFlightTable.put(5.0, 1.15);
        timeOfFlightTable.put(5.45, 1.12);
        timeOfFlightTable.put(6.0, 1.26);
        timeOfFlightTable.put(6.5, 1.27);

        // ---- Feed shooter speed table (tune on practice field) ----
        feedShooterSpeedTable.put(5.0,  44.0);
        feedShooterSpeedTable.put(6.0,  46.0);
        feedShooterSpeedTable.put(7.0,  48.5);
        feedShooterSpeedTable.put(8.0,  51.0);
        feedShooterSpeedTable.put(9.0,  53.5);
        feedShooterSpeedTable.put(10.0, 56.0);
        feedShooterSpeedTable.put(11.0, 58.5);
        feedShooterSpeedTable.put(12.0, 61.0);

        // ---- Feed time of flight table (tune on practice field) ----
        feedTimeOfFlightTable.put(5.0,  1.30);
        feedTimeOfFlightTable.put(6.0,  1.45);
        feedTimeOfFlightTable.put(7.0,  1.60);
        feedTimeOfFlightTable.put(8.0,  1.75);
        feedTimeOfFlightTable.put(9.0,  1.90);
        feedTimeOfFlightTable.put(10.0, 2.05);
        feedTimeOfFlightTable.put(11.0, 2.20);
        feedTimeOfFlightTable.put(12.0, 2.35);
    }

    // =========================================================================
    // Hub calculations (original method signatures and logic, unchanged)
    // =========================================================================

    public double calculateToF(double distance) {
        return timeOfFlightTable.get(distance) - ToFInterpAdjustment;
    }

    public double calculateShooterSpeed(double distance) {
        return shooterSpeedTable.get(distance) + shooterInterpSpeedAdjustment;
    }

    public double calculateHoodPosition(double distance) {
        double hood = 0.0;
        if (distance <= 1.5) {
            hood = 0;
        } else if (distance >= 5.5){
            hood = 0.625;
        } else {
            hood = hoodTable.get(distance);
        }
        return hood;
    }

    // =========================================================================
    // Feed calculations (new)
    // =========================================================================

    public double calculateFeedShooterSpeed(double distance) {
        return feedShooterSpeedTable.get(distance);
    }

    public double calculateFeedToF(double distance) {
        return feedTimeOfFlightTable.get(distance);
    }

    /** Hood is always at maximum for feed shots. */
    public double calculateFeedHoodPosition() {
        return FEED_HOOD_MAX;
    }

    // =========================================================================
    // Shot-on-the-move calculations (moved from ShooterSubsystem, logic unchanged)
    // =========================================================================

    /**
     * Hub virtual target - uses hub ToF table. Original logic, unchanged.
     * Called by getVirtualTarget when robot is moving.
     */
    public Translation2d calculateVirtualTarget(double realTargetX, double realTargetY) {
        double ToFFudgeFactor = 1;
        
        double[] robotData = calculateRobotMetric();

        // Gets the xFinal and yFinal of the robot
        Translation2d shooterPos = new Translation2d(robotData[0], robotData[1]);
        // Gets the vxFinal and vyFinal of the robot
        double velocityX = robotData[2];
        double velocityY = robotData[3];

        Translation2d virtualTarget = new Translation2d(realTargetX, realTargetY);
        double distance = shooterPos.getDistance(virtualTarget);
        double timeOfFlight = calculateToF(distance);
        double virtualX = realTargetX;
        double virtualY = realTargetY;

        for (int i = 0; i < 7; i++) {
            virtualX = realTargetX - (velocityX * (timeOfFlight)) * ToFFudgeFactor;
            virtualY = realTargetY - (velocityY * (timeOfFlight)) * ToFFudgeFactor;
            

            Translation2d testTarget = new Translation2d(virtualX, virtualY);
            distance = shooterPos.getDistance(testTarget);
            timeOfFlight = calculateToF(distance);
        }
        return new Translation2d(virtualX, virtualY);
    }

    /**
     * Feed virtual target - same algorithm as hub but uses feed ToF table
     * to correctly lead for the longer lob arc.
     */
    public Translation2d calculateFeedVirtualTarget(double realTargetX, double realTargetY) {
        double ToFFudgeFactor = 1;

        double[] robotData = calculateRobotMetric();

        Translation2d shooterPos = new Translation2d(robotData[0], robotData[1]);
        double velocityX = robotData[2];
        double velocityY = robotData[3];

        Translation2d virtualTarget = new Translation2d(realTargetX, realTargetY);
        double distance = shooterPos.getDistance(virtualTarget);
        double timeOfFlight = calculateFeedToF(distance);
        double virtualX = realTargetX;
        double virtualY = realTargetY;

        for (int i = 0; i < 7; i++) {
            virtualX = realTargetX - (velocityX * (timeOfFlight)) * ToFFudgeFactor;
            virtualY = realTargetY - (velocityY * (timeOfFlight)) * ToFFudgeFactor;

            Translation2d testTarget = new Translation2d(virtualX, virtualY);
            distance = shooterPos.getDistance(testTarget);
            timeOfFlight = calculateFeedToF(distance);
        }
        return new Translation2d(virtualX, virtualY);
    }

    public double[] calculateRobotMetric(){
        double deltaT = 0.020;

        double[] variables = new double[4];

        double xInitial = (drivetrain.getPose().plus(shooterOffset)).getTranslation().getX();
        double yInitial = (drivetrain.getPose().plus(shooterOffset)).getTranslation().getY();
        double vxInitial = drivetrain.getFieldRelativeSpeeds().vxMetersPerSecond;
        double vyInitial = drivetrain.getFieldRelativeSpeeds().vyMetersPerSecond;
        // Gets the accelerations of the robot
        double accelerationX = calculateRobotAcceleration()[0];
        double accelerationY = calculateRobotAcceleration()[1];

        double vxFinal = vxInitial + (accelerationX * deltaT);
        double vyFinal = vyInitial + (accelerationY * deltaT);

        double xFinal = xInitial + (vxInitial * deltaT) + (0.5 * accelerationX * deltaT * deltaT);
        double yFinal = yInitial + (vyInitial * deltaT) + (0.5 * accelerationY * deltaT * deltaT);

        variables[0] = xFinal;
        variables[1] = yFinal;
        variables[2] = vxFinal;
        variables[3] = vyFinal;

        return variables;
    }

    public double[] calculateRobotAcceleration(){
        // G is 9.80665
        Pigeon2 pigeon = drivetrain.getPigeon2();

        double robotAx = pigeon.getAccelerationX().getValueAsDouble() * 9.80665;
        double robotAy = pigeon.getAccelerationY().getValueAsDouble() * 9.80665;

        Rotation2d yaw = pigeon.getRotation2d();
        double yawCos = yaw.getCos();
        double yawSin = yaw.getSin();

        double fieldAx = robotAx * yawCos - robotAy * yawSin;
        double fieldAy = robotAx * yawSin + robotAy * yawCos;

        return new double[]{fieldAx, fieldAy};
    }

    /** Hub virtual target gate - original logic, unchanged. */
    public Translation2d getVirtualTarget(double targetX, double targetY) {
        if(robotVelocitySubscriber.getAsDouble() < 0.1){
            return new Translation2d(targetX, targetY);
        }
        return calculateVirtualTarget(targetX, targetY);
    }

    /** Feed virtual target gate - same stationary check, routes to feed solver. */
    public Translation2d getFeedVirtualTarget(double targetX, double targetY) {
        if(robotVelocitySubscriber.getAsDouble() < 0.1){
            return new Translation2d(targetX, targetY);
        }
        return calculateFeedVirtualTarget(targetX, targetY);
    }
}