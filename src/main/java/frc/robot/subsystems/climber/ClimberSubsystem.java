package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    // private Solenoid climberSolenoid;
    // private Compressor climberCompressor;
    private TalonFX climberMotor;

    private DutyCycleOut dcOut = new DutyCycleOut(0);
    private VelocityVoltage velocityOut = new VelocityVoltage(0);

    public ClimberSubsystem() {
        climberMotor = new TalonFX(Constants.climberConstants.CLIMBER_MOTOR_ID);

        MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(200);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration().withMotorOutput(outputConfig)
                .withCurrentLimits(currentLimitConfig);

        climberMotor.getConfigurator().apply(motorConfig);

        // climberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        // climberCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
        // climberCompressor.enableDigital();
    }

    public void climberUp() {
        climberMotor.setControl(dcOut.withOutput(0.1));
    }

    public void climberDown() {
        climberMotor.setControl(dcOut.withOutput(0.1));
    }

    // @Override
    // public void periodic() {
    // // TODO Auto-generated method stub
    // super.periodic();
    // }

    // public void climberUp() {
    // climberSolenoid.set(true);
    // }

    // public void climberdown() {
    // climberSolenoid.set(false);
    // }

}
