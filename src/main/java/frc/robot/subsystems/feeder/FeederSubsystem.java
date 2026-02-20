package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

    private TalonFX feederMotor;

    private DutyCycleOut dcOut = new DutyCycleOut(0);
    private VelocityVoltage velocityOut = new VelocityVoltage(0);

    private double statorCurrentLimit = 80.0;
    private StatusSignal spindexerVelocitySS;
    private StatusSignal spindexerVelocityErrorSS;
    private StatusSignal spindexerStatorCurrentSS;

    private BooleanPublisher spindexerStallPublisher;

    public FeederSubsystem() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable robotTable = inst.getTable("Robot Data");
        NetworkTable feederTable = robotTable.getSubTable("Feeder Subsystem");
        spindexerStallPublisher = feederTable.getBooleanTopic("Spindexer Stall").publish();
        spindexerStallPublisher.set(false);

        feederMotor = new TalonFX(Constants.feederConstants.FEEDER_MOTOR_ID);

        ConfigureFeederMotor();
    }

    private void ConfigureFeederMotor() {

        MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(80);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration().withMotorOutput(outputConfig)
                .withCurrentLimits(currentLimitConfig);

        feederMotor.getConfigurator().apply(motorConfig);
    }

    public void runFeeder() {
        feederMotor.setControl(dcOut.withOutput(0.8));
    }

    public void stopFeeder() {
        feederMotor.setControl(dcOut.withOutput(0.0));
    }

}
