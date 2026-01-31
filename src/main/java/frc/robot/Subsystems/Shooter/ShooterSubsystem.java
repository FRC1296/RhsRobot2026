package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    private TalonFX shooterHoodMotor;    // Will need to be changed to minion syntax
    private TalonFX shooterMasterMotor;
    private TalonFX shooterFollowerMotor;
    private TalonFX shooterFeederMotor;

    private double SHOOTER_SPEED = 0.0; // May need to change

    public ShooterSubsystem(){
        shooterHoodMotor = new TalonFX(79);
        shooterMasterMotor = new TalonFX(69);
        shooterFollowerMotor = new TalonFX(59);
        shooterFeederMotor = new TalonFX(49);

        TalonFXConfiguration shooterHoodConfig = new TalonFXConfiguration();
        shooterHoodMotor.getConfigurator().apply(shooterHoodConfig);
        TalonFXConfiguration shooterMasterConfig = new TalonFXConfiguration();
        shooterMasterMotor.getConfigurator().apply(shooterMasterConfig);
        TalonFXConfiguration shooterFollowerConfig = new TalonFXConfiguration();
        shooterFollowerMotor.getConfigurator().apply(shooterFollowerConfig);
        TalonFXConfiguration shooterFeederConfig = new TalonFXConfiguration();
        shooterFeederMotor.getConfigurator().apply(shooterFeederConfig);

        ConfigureShooterHoodMotor();
        ConfigureShooterMasterMotor();
        ConfigureShooterFollowerMotor();
        ConfigureShooterFeederMotor();
    }

    public void ConfigureShooterHoodMotor(){
        MotorOutputConfigs outputConfig =
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(60); // Change later

        TalonFXConfiguration shooterHoodMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig);

         shooterHoodMotor.getConfigurator().apply(shooterHoodMotorConfig);
         shooterHoodMotor.setPosition(0);
    }

    public void ConfigureShooterMasterMotor(){
        MotorOutputConfigs outputConfig =
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(60); // Change later

        TalonFXConfiguration shooterMasterMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig);

         shooterMasterMotor.getConfigurator().apply(shooterMasterMotorConfig);
         shooterMasterMotor.setPosition(0);
    }
    
    public void ConfigureShooterFollowerMotor(){
        MotorOutputConfigs outputConfig =
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(60); // Change later

        TalonFXConfiguration shooterFollowerMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig);

         shooterFollowerMotor.getConfigurator().apply(shooterFollowerMotorConfig);
         shooterFollowerMotor.setPosition(0);
    }

    public void ConfigureShooterFeederMotor(){
        MotorOutputConfigs outputConfig =
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(60); // Change later

        TalonFXConfiguration shooterFeederMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig);

         shooterFeederMotor.getConfigurator().apply(shooterFeederMotorConfig);
         shooterFeederMotor.setPosition(0);
    }



}
