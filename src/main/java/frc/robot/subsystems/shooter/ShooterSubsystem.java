package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    //private CANcoder hoodAbsEncoder;

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    private DutyCycleOut dcOut = new DutyCycleOut(0);
    private VelocityVoltage velocityOut = new VelocityVoltage(0);

    private DoublePublisher shooterSpeedPublisher;
    private DoubleSubscriber robotVelocitySubscriber;

    private double shooterSpeed = 35.0;

    private double hoodCruiseVelocity = 100;
    private final double hoodkP = 20.0;
    private final double hoodkI = 0.0;
    private final double hoodkD = 0.0;
    private final double hoodkS = 0.37;
    private final double hoodkV = 0.055;

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
        //hoodAbsEncoder = new CANcoder(Constants.shooterConstants.HOOD_ENCODER_ID);

        ConfigureAbsoluteEncoder();
        ConfigureHoodMotor();
        ConfigureShooterMotors();
    }

    private void configureNetworkTable() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable robotTable = inst.getTable(Constants.NETWORK_TABLE);
        NetworkTable shooterTable = robotTable.getSubTable(Constants.NT_SHOOTER);
        shooterSpeedPublisher = shooterTable.getDoubleTopic(Constants.NT_SHOOTER_VELOCITY).publish();

        NetworkTable driveTable = robotTable.getSubTable(Constants.NT_DRIVE);
        robotVelocitySubscriber = driveTable.getDoubleTopic(Constants.NT_DRIVE_VELOCITY).subscribe(0.0);
    }

    private void ConfigureAbsoluteEncoder() {
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cc_cfg.MagnetSensor.withMagnetOffset(Rotations.of(0.0));
        //hoodAbsEncoder.getConfigurator().apply(cc_cfg);
        //hoodAbsEncoder.clearStickyFaults();
        //hoodAbsEncoder.setPosition(0.0);
    }

    private void ConfigureHoodMotor() {
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(120);

        Slot0Configs slotZeroConfigs = new Slot0Configs()
                .withKS(hoodkS)
                .withKP(hoodkP)
                .withKI(hoodkI)
                .withKD(hoodkD)
                .withKV(hoodkV);

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(hoodCruiseVelocity)
                .withMotionMagicAcceleration(hoodCruiseVelocity)
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
                .withKP(0.3)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.34)
                .withKV(0.118);

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

        //hoodPositionPublisher.set(hoodAbsEncoder.getAbsolutePosition().getValueAsDouble());
        shooterSpeedPublisher.set(shooterSpeed);
    }

    protected void resetMotorEncoder() {
        //TODO : implement 
    }

    public double getShooterVelocity(){
        return shooterMasterMotor.getVelocity().getValueAsDouble();
    }

    public void increaseShooterSpeed() {
        shooterSpeed += 0.5;
    }

    public void decreaseShooterSpeed() {
        shooterSpeed -= 0.5;
    }

    public void setShooterSafeVelocity() {
        shooterMasterMotor.setControl(velocityOut.withSlot(0).withVelocity(37));
    }

    public void increaseShooterInterpSpeed() {
        shooterInterpSpeedAdjustment = shooterInterpSpeedAdjustment + 0.5;
    }

    public void decreaseShooterInterpSpeed() {
        shooterInterpSpeedAdjustment = shooterInterpSpeedAdjustment - 0.5;
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

    public void setAutoShooter(double targetX, double targetY) {
        Pose2d drivetrainPose = drivetrain.getPose();
        Translation2d shooterTranslation = (drivetrainPose.plus(shooterOffset)).getTranslation();
        double distanceToHub = shooterTranslation.getDistance(new Translation2d(targetX, targetY));

        shooterMasterMotor.setControl(velocityOut.withSlot(0).withVelocity(calculateShooterSpeed(distanceToHub)));
        hoodMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(calculateHoodPosition(distanceToHub)));
        }

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

    public void moveHoodToZero() {
    hoodMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(0.0));
}

    public Transform2d getShooterOffset() {
        return shooterOffset;
    }
}
