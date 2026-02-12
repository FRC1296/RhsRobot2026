package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
 * Deployed position = -0.047119 rotations Undeployed position = 0.312988 rotations
 */

public class IntakeSubsystem extends SubsystemBase {

    private TalonFX intakeRollerMotor;
    private TalonFX intakeDeployMotor;
    private CANcoder intakeAbsEncoder;

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    private DutyCycleOut dcOut = new DutyCycleOut(0);

    private DoublePublisher intakePositionPublisher;

    private double intakeDeployPosition = 0.68;
    private double intakeUndeployPosition = 0.1;

    private double deployCruiseVelocity = 10;

    private final double deploykP = 10.0;
    private final double deploykI = 0.0;
    private final double deploykD = 0.0;
    private final double deploykG = 0.0;

    public IntakeSubsystem() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable limebotTable = inst.getTable("Robot Data");
        NetworkTable shooterTable = limebotTable.getSubTable("Intake Subsystem");
        intakePositionPublisher = shooterTable.getDoubleTopic("Intake Position").publish();

        intakeRollerMotor = new TalonFX(Constants.intakeConstants.INTAKE_ROLLER_MOTOR_ID);
        intakeDeployMotor = new TalonFX(Constants.intakeConstants.INTAKE_DEPLOY_MOTOR_ID);
        intakeAbsEncoder = new CANcoder(Constants.intakeConstants.INTAKE_ENCODER_ID);

        ConfigureAbsoluteEncoder();
        ConfigureIntakeRollerMotor();
        ConfigureDeployMotor();
    }

    // TODO: Validate these settings for the configuration, read documentation on method by hovering over method name
    private void ConfigureAbsoluteEncoder() {
        /* Configure CANcoder to zero the magnet appropriately */
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.withMagnetOffset(Rotations.of(0));

        intakeAbsEncoder.getConfigurator().apply(cc_cfg);
    }

    private void ConfigureIntakeRollerMotor() {
        MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(80);
        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration().withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig);

        intakeRollerMotor.getConfigurator().apply(intakeMotorConfig);
    }

    private void ConfigureDeployMotor() {

        MotorOutputConfigs outputConfig = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(270);

        Slot0Configs slotZeroConfigs = new Slot0Configs()
            .withKG(deploykG)
            .withKP(deploykP)
            .withKI(deploykI)
            .withKD(deploykD);

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(deployCruiseVelocity)
            .withMotionMagicAcceleration(deployCruiseVelocity * 2).withMotionMagicJerk(0);

        //TODO : need to validate this configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withFusedCANcoder(intakeAbsEncoder);

        TalonFXConfiguration intakeDeployMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig)
            .withSlot0(slotZeroConfigs).withMotionMagic(mmConfigs)
            .withFeedback(feedbackConfigs);

        intakeDeployMotor.getConfigurator().apply(intakeDeployMotorConfig);
        intakeDeployMotor.setPosition(0);
    }

    public void periodic() {
        intakePositionPublisher.set(getIntakePosition());
    }

    public void runIntake() {
        intakeRollerMotor.setControl(dcOut.withOutput(0.80));
    }

    public void stopIntake() {
        intakeRollerMotor.setControl(dcOut.withOutput(0.0));
    }

    public double getIntakePosition() {
        return intakeDeployMotor.getPosition().getValueAsDouble();
    }

    public void undeployIntake() {
        // TODO : Validate the absolute sensor position
        //intakeDeployMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(intakeUndeployPosition));
    }

    public void deployIntake() {
        // TODO : Validate the absolute sensor position
        //intakeDeployMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(intakeDeployPosition));
    }
}
