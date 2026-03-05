package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class ShooterSubsystem extends ShooterInterpolationHelper {

    private TalonFX hoodMotor;
    private TalonFX shooterMasterMotor;
    private TalonFX shooterFollowerMotor;
    private CANcoder hoodAbsEncoder;

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    private DutyCycleOut dcOut = new DutyCycleOut(0);
    private VelocityVoltage velocityOut = new VelocityVoltage(0);

    private DoublePublisher hoodPositionPublisher;
    private DoublePublisher shooterSpeedPublisher;
    private DoubleSubscriber robotVelocitySubscriber;
    private DoubleSubscriber robotDistanceToHubSubscriber;

    private double shooterSpeed = 35.0;

    // Hood Absolute Encoder positions
    private double hoodPos = 0.05;

    // Hood Motor Encoder Positions (zero down at start)
    private double hoodStartPosition = 0.0;
    private double hoodUp100Percent = 0.70;
    private double hoodUp50Percent = 0.35;

    private double hoodCruiseVelocity = 10;
    private final double hoodkP = 5.0;
    private final double hoodkI = 0.0;
    private final double hoodkD = 0.0;
    private final double hoodkS = 0.3;
    private final double hoodkV = 0.05;

    private Transform2d shooterOffset = new Transform2d(new Translation2d(0.0, 0.0381),new Rotation2d());
    private CommandSwerveDrivetrain drivetrain;

    private StatusSignal<Boolean> hoodMMAtSetpoint;
    private StatusSignal<Voltage> hoodVoltage;
    private StatusSignal<Boolean> hoodMMEnabled;

    public ShooterSubsystem(CommandSwerveDrivetrain drive) {
        super("Shooter");

        configureNetworkTable();

        drivetrain = drive;

        hoodMotor = new TalonFX(Constants.shooterConstants.HOOD_MOTOR_ID);
        shooterMasterMotor = new TalonFX(Constants.shooterConstants.SHOOTER_MASTER_MOTOR_ID);
        shooterFollowerMotor = new TalonFX(Constants.shooterConstants.SHOOTER_SLAVE_MOTOR_ID);
        hoodAbsEncoder = new CANcoder(Constants.shooterConstants.HOOD_ENCODER_ID);

        ConfigureAbsoluteEncoder();
        ConfigureHoodMotor();
        ConfigureShooterMotors();
    }

    private void configureNetworkTable() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable robotTable = inst.getTable(Constants.NETWORK_TABLE);
        NetworkTable shooterTable = robotTable.getSubTable(Constants.NT_SHOOTER);
        hoodPositionPublisher = shooterTable.getDoubleTopic(Constants.NT_SHOOTER_HOOD_POSITION).publish();
        shooterSpeedPublisher = shooterTable.getDoubleTopic(Constants.NT_SHOOTER_VELOCITY).publish();

        NetworkTable driveTable = robotTable.getSubTable(Constants.NT_DRIVE);
        robotVelocitySubscriber = driveTable.getDoubleTopic(Constants.NT_DRIVE_VELOCITY).subscribe(0.0);

        robotDistanceToHubSubscriber = robotTable.getDoubleTopic(Constants.NT_ROBOT_DISTANCE_TO_HUB).subscribe(0.0);
    }

    private void ConfigureAbsoluteEncoder() {
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cc_cfg.MagnetSensor.withMagnetOffset(Rotations.of(0.0));
        hoodAbsEncoder.getConfigurator().apply(cc_cfg);
        hoodAbsEncoder.clearStickyFaults();
        hoodAbsEncoder.setPosition(0.0);
    }

    private void ConfigureHoodMotor() {
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(60);

        Slot0Configs slotZeroConfigs = new Slot0Configs()
                .withKS(hoodkS)
                .withKP(hoodkP)
                .withKI(hoodkI)
                .withKD(hoodkD)
                .withKV(hoodkV);

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(hoodCruiseVelocity)
                .withMotionMagicAcceleration(hoodCruiseVelocity / 2)
                .withMotionMagicJerk(0);

        // FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        //         .withFeedbackRemoteSensorID(Constants.shooterConstants.HOOD_ENCODER_ID)
        //         .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        //         .withSensorToMechanismRatio(1.0)
        //         .withRotorToSensorRatio(1.0);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withMotorOutput(outputConfigs)
                .withCurrentLimits(currentConfigs)
                .withSlot0(slotZeroConfigs)
                .withMotionMagic(mmConfigs);
                //.withFeedback(feedbackConfigs);

        hoodMotor.getConfigurator().apply(motorConfig);
        hoodMotor.setPosition(0);
        hoodMotor.clearStickyFaults();

        hoodMMAtSetpoint = hoodMotor.getMotionMagicAtTarget();
        hoodMMEnabled = hoodMotor.getMotionMagicIsRunning();
        hoodVoltage = hoodMotor.getMotorVoltage();
    }

    private void ConfigureShooterMotors() {
        shooterFollowerMotor.setControl(
                new Follower(Constants.shooterConstants.SHOOTER_MASTER_MOTOR_ID, MotorAlignmentValue.Opposed));

        MotorOutputConfigs outputConfig = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(120);

        Slot0Configs slotZeroConfigs = new Slot0Configs()
                .withKP(0.4)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.2)
                .withKV(0.116);

        TalonFXConfiguration masterMotorConfig = new TalonFXConfiguration()
                .withCurrentLimits(currentConfigs)
                .withMotorOutput(outputConfig).withSlot0(slotZeroConfigs);

        TalonFXConfiguration followerMotorConfig = new TalonFXConfiguration()
                .withCurrentLimits(currentConfigs).withSlot0(slotZeroConfigs);

        shooterMasterMotor.getConfigurator().apply(masterMotorConfig);
        shooterFollowerMotor.getConfigurator().apply(followerMotorConfig);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(hoodMMAtSetpoint, hoodMMEnabled, hoodVoltage);
        if (hoodMMEnabled.getValue() && hoodMMAtSetpoint.getValue()) {
            resetMotorEncoder();
        }

        hoodPositionPublisher.set(hoodAbsEncoder.getAbsolutePosition().getValueAsDouble());
        shooterSpeedPublisher.set(shooterSpeed);
    }

    protected void resetMotorEncoder() {
        //TODO : implement 
    }

    // public void fullCourtShoot() {
    //     shooterMasterMotor.setControl(velocityOut.withSlot(0).withVelocity(95.0));
    //     hoodMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(0.5));
    // }

    public double getShooterVelocity(){
        return shooterMasterMotor.getVelocity().getValueAsDouble();
    }

    public void increaseShooterSpeed() {
        shooterSpeed += 1.0;
    }

    public void decreaseShooterSpeed() {
        shooterSpeed -= 1.0;
    }

    public void setShooterSafeVelocity() {
        shooterMasterMotor.setControl(velocityOut.withSlot(0).withVelocity(37));
    }

    public void increaseShooterInterpSpeed() {
        shooterInterpSpeedAdjustment = shooterInterpSpeedAdjustment + 2;
    }

    public void decreaseShooterInterpSpeed() {
        shooterInterpSpeedAdjustment = shooterInterpSpeedAdjustment - 2;
    }

    public void increaseToF() {
        ToFInterpAdjustment = ToFInterpAdjustment + 0.05;
    }

    public void decreaseToF() {
        ToFInterpAdjustment = ToFInterpAdjustment - 0.05;
    }
    
    public void runMasterShooter() {
        shooterMasterMotor.setControl(velocityOut.withSlot(0).withVelocity(shooterSpeed));
    }

    public void stopMasterShooter() {
        shooterMasterMotor.setControl(dcOut.withOutput(0.0));
    }

    public void moveHoodUp() {
        hoodMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(hoodUp50Percent));
    }

    public void moveHoodZero() {
        hoodMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(hoodStartPosition));
    }

    public void runHood() {
        hoodPos += 0.05;
        hoodMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(hoodPos));
    }

    public void reverseHood() {
        if ((hoodPos - 0.05) > 0) {
            hoodPos -= 0.05;
            hoodMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(hoodPos));
        }
    }

    public void setAutoShooter(double targetX, double targetY) {
        Translation2d virtualTarget = new Translation2d(targetX, targetY);
        if(robotVelocitySubscriber.getAsDouble() > 0.1){
            virtualTarget = calculateVirtualTarget(targetX, targetY);
        }

        Pose2d drivetrainPose = drivetrain.getPose();
        Translation2d shooterTranslation = (drivetrainPose.plus(shooterOffset)).getTranslation();
        double distanceToHub = shooterTranslation.getDistance(virtualTarget);

        shooterMasterMotor.setControl(velocityOut.withSlot(0).withVelocity(calculateShooterSpeed(distanceToHub)));
        hoodMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(calculateHoodPosition(distanceToHub)));
        }

    public Translation2d calculateVirtualTarget(double realTargetX, double realTargetY) {
        Pose2d pose = drivetrain.getPose();
        Translation2d shooterPos = (pose.plus(shooterOffset)).getTranslation();
        ChassisSpeeds speeds = drivetrain.getFieldRelativeSpeeds();
        Translation2d virtualTarget = new Translation2d(realTargetX, realTargetY);
        double distance = shooterPos.getDistance(virtualTarget);
        double timeOfFlight = calculateToF(distance);
        double virtualX = realTargetX;
        double virtualY = realTargetY;


        for (int i = 0; i < 5; i++) {
            distance = shooterPos.getDistance(virtualTarget);
            timeOfFlight = calculateToF(distance);

            virtualX = virtualX - (speeds.vxMetersPerSecond * timeOfFlight);
            virtualY = virtualY - (speeds.vyMetersPerSecond * timeOfFlight);

            virtualTarget = new Translation2d(virtualX, virtualY);
        }

        return new Translation2d(virtualX, virtualY);
    }

    public Translation2d getVirtualTarget(double targetX, double targetY) {
        if(robotVelocitySubscriber.getAsDouble() < 0.1){
            return new Translation2d(targetX, targetY);
        }
        return calculateVirtualTarget(targetX, targetY);
    }

    public void shooterAutoInterpolateBool(boolean bool) {
        Constants.shooterConstants.shooterInterpolate = bool;
    }

    public void stopAutoAimAndShoot() {
        Constants.shooterConstants.shooterInterpolate = false;
        Constants.turretConstants.turretAimAtHub = false;
    }

    /*
     * The shooter is not centered on the robot, so this will return the transform
     * necessary for adjusting the pose.
     * 
     * Primary used for calculating distances
     * 
     */
    public Transform2d getShooterOffset() {
        return shooterOffset;
    }
}
