package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

    private TalonFX spindexerMotor;
    private TalonFX feederMotor;

    private DutyCycleOut dcOut = new DutyCycleOut(0);

    public FeederSubsystem() {

        spindexerMotor = new TalonFX(Constants.feederConstants.SPINDEXER_MOTOR_ID);
        feederMotor = new TalonFX(Constants.feederConstants.FEEDER_MOTOR_ID);

        ConfigureSpindexerMotor();
        ConfigureFeederMotor();
    }

    private void ConfigureSpindexerMotor() {

        MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(40);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration().withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig);

        spindexerMotor.getConfigurator().apply(motorConfig);
    }

    private void ConfigureFeederMotor() {

        MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(80);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration().withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig);

        feederMotor.getConfigurator().apply(motorConfig);
    }

    public void runFeeders() {
        feederMotor.setControl(dcOut.withOutput(0.6));
        spindexerMotor.setControl(dcOut.withOutput(0.15));
    }

    public void stopFeeders() {
        feederMotor.setControl(dcOut.withOutput(0.0));
        spindexerMotor.setControl(dcOut.withOutput(0.0));
    }

}
