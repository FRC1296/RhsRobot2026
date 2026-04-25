package frc.robot.subsystems.intake;

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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFX intakeRollerMotorLeft;
    private TalonFX intakeRollerMotorRight;
    private TalonFX intakeDeployMotor;
    private CANcoder intakeAbsEncoder;

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    private VelocityVoltage velocityOut = new VelocityVoltage(0);
    private DutyCycleOut dcOut = new DutyCycleOut(0);

    private double intakeDeployPosition = -0.001;
    private double intakeStowPosition = -0.344;
    private double intakeUndeployPosition = 5.0;
    private double intakeAgitatePosition = 5.0;

    private double intakeDeploySpeed = 0.10;

    private double deployCruiseVelocity = 100;
    private double deployCruiseAcceleration = 75;
    private double deployCruiseJerk = 0;
    

    private final double deploykP = 20;
    private final double deploykI = 0.0;
    private final double deploykD = 0.0;
    private final double deploykG = -0.52;
    private final double deployKS = 0.45;
    private final double deployKV = 0.145;

    private StatusSignal<Boolean> deployMMAtSetpoint;
    private StatusSignal<Voltage> deployVoltage;
    private StatusSignal<Boolean> deployMMEnabled;

    public IntakeSubsystem() {
        super("Intake");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable robotTable = inst.getTable(Constants.NETWORK_TABLE);
        NetworkTable shooterTable = robotTable.getSubTable(Constants.NT_INTAKE);

        intakeRollerMotorLeft = new TalonFX(Constants.intakeConstants.INTAKE_ROLLER_MOTOR_LEFT_ID);
        intakeRollerMotorRight = new TalonFX(Constants.intakeConstants.INTAKE_ROLLER_MOTOR_RIGHT_ID);
        intakeDeployMotor = new TalonFX(Constants.intakeConstants.INTAKE_DEPLOY_MOTOR_ID);
        intakeAbsEncoder = new CANcoder(Constants.intakeConstants.INTAKE_ENCODER_ID);

        ConfigureAbsoluteEncoder();
        ConfigureIntakeRollerMotor();
        ConfigureDeployMotor();
    }

    private void ConfigureAbsoluteEncoder() {
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.withMagnetOffset(0.0);
        intakeAbsEncoder.getConfigurator().apply(cc_cfg);
    }

    private void ConfigureIntakeRollerMotor() {
        intakeRollerMotorRight.setControl(
                new Follower(Constants.intakeConstants.INTAKE_ROLLER_MOTOR_LEFT_ID, MotorAlignmentValue.Opposed));

        MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(120);
        Slot0Configs slotZeroConfigs = new Slot0Configs()
                .withKP(0.6)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.25)
                .withKV(0.112);

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration()
                .withCurrentLimits(currentLimitConfig)
                .withMotorOutput(outputConfig).withSlot0(slotZeroConfigs);

        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration()
                .withCurrentLimits(currentLimitConfig).withSlot0(slotZeroConfigs);

        intakeRollerMotorLeft.getConfigurator().apply(leftMotorConfig);
        intakeRollerMotorRight.getConfigurator().apply(rightMotorConfig);
    }

    private void ConfigureDeployMotor() {

        MotorOutputConfigs outputConfig = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(55);

        Slot0Configs slotZeroConfigs = new Slot0Configs()
                .withKG(deploykG)
                .withKP(deploykP)
                .withKI(deploykI)
                .withKD(deploykD)
                .withKS(deployKS)
                .withKV(deployKV)
                .withGravityType(GravityTypeValue.Arm_Cosine);

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(deployCruiseVelocity)
                .withMotionMagicAcceleration(deployCruiseAcceleration).withMotionMagicJerk(deployCruiseJerk);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(Constants.intakeConstants.INTAKE_ENCODER_ID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(1.0)
                .withRotorToSensorRatio(28.166666666);

        TalonFXConfiguration intakeDeployMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig)
                .withSlot0(slotZeroConfigs).withMotionMagic(mmConfigs)
                .withFeedback(feedbackConfigs);

        intakeDeployMotor.getConfigurator().apply(intakeDeployMotorConfig);
        //intakeDeployMotor.setPosition(intakeStowPosition);

        deployMMAtSetpoint = intakeDeployMotor.getMotionMagicAtTarget();
        deployMMEnabled = intakeDeployMotor.getMotionMagicIsRunning();
        deployVoltage = intakeDeployMotor.getMotorVoltage();
    }

    public void periodic() {
    }

    public void runIntake() {
        intakeRollerMotorLeft.setControl(velocityOut.withVelocity(75.0));
    }

    public void stopIntake() {
        intakeRollerMotorLeft.setControl(dcOut.withOutput(0.0));
    }

    public void runIntakeReverse() {
        intakeRollerMotorLeft.setControl(velocityOut.withVelocity(-75.0));
    }

    public void undeployIntake() {
        intakeDeployMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(intakeUndeployPosition));
    }

    public void stopDeployIntake() {
        intakeDeployMotor.setControl(dcOut.withOutput(0));
    }

    public void deployIntake() {
        intakeDeployMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(intakeDeployPosition));
    }

    public void moveIntakeToAgitate() {
        intakeDeployMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(intakeAgitatePosition));
    }

     public void manuelUndeployIntake() {
        intakeDeployMotor.setControl(dcOut.withOutput(-intakeDeploySpeed));
    }

     public void manuelDeployIntake() {
        intakeDeployMotor.setControl(dcOut.withOutput(intakeDeploySpeed));
    }

    public void resetDeployPosition() {
        //intakeDeployMotor.setPosition(intakeDeployPosition);
    }
}