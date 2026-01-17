package frc.robot.Subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private TalonFX intakeMotor;
    private TalonFX intakeDeployMotor;

    private DutyCycleOut dcOut = new DutyCycleOut(0);

    
    // Constants undecided as of 1/16/26
    private double INTAKE_SPEED = 0.0;
    private double DEPLOY_SPEED = 0.0;

    public IntakeSubsystem(){
        intakeMotor = new TalonFX(99);
        intakeDeployMotor = new TalonFX(89);


        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeMotor.getConfigurator().apply(intakeConfig);
        TalonFXConfiguration intakeDeployConfig = new TalonFXConfiguration();
        intakeDeployMotor.getConfigurator().apply(intakeDeployConfig);

        ConfigureIntakeMotor();
        ConfigureDeployMotor();
    }

    private void ConfigureIntakeMotor(){
        
        MotorOutputConfigs outputConfig =
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(60); // Change later

        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig);

         intakeMotor.getConfigurator().apply(intakeMotorConfig);
         intakeMotor.setPosition(0);
    }

    private void ConfigureDeployMotor(){

        MotorOutputConfigs outputConfig =
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(60); // Change later

        TalonFXConfiguration intakeDeployMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig);

         intakeMotor.getConfigurator().apply(intakeDeployMotorConfig);
         intakeMotor.setPosition(0);
    }

     public void runIntake() {
         intakeMotor.setControl(dcOut.withOutput(1.0));
    }

    public void stopIntake() {
         intakeMotor.setControl(dcOut.withOutput(0.0));
    }
}
