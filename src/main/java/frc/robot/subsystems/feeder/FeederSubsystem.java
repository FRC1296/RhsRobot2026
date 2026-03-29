package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class FeederSubsystem extends SubsystemBase {

    private TalonFX feederMotor;

    private VelocityVoltage velocityOut = new VelocityVoltage(0);

    private DutyCycleOut dcOut = new DutyCycleOut(0);

    private CommandSwerveDrivetrain drivetrain;

    public FeederSubsystem(CommandSwerveDrivetrain drivetrain) {
        super("Feeder");
        feederMotor = new TalonFX(Constants.feederConstants.FEEDER_MOTOR_ID);
        this.drivetrain = drivetrain;
        ConfigureFeederMotor();
    }

    private void ConfigureFeederMotor() {

        MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(80);

        Slot0Configs slotZeroConfigs = new Slot0Configs()
                .withKP(0.65)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.4)
                .withKV(0.096);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration().withMotorOutput(outputConfig)
                .withCurrentLimits(currentLimitConfig).withSlot0(slotZeroConfigs);

        feederMotor.getConfigurator().apply(motorConfig);
    }

    public void runFeeder() {
        // ChassisSpeeds speed = drivetrain.getRobotRelativeSpeeds();
        // double currentSpeed = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
        // if(currentSpeed <= 1.2 && DriverStation.isTeleop()){
        //     feederMotor.setControl(velocityOut.withSlot(0).withVelocity(Constants.feederConstants.feederSpeed));
        // } else if(DriverStation.isAutonomous()){
        //     feederMotor.setControl(velocityOut.withSlot(0).withVelocity(Constants.feederConstants.feederSpeed));
        // }
        feederMotor.setControl(velocityOut.withSlot(0).withVelocity(Constants.feederConstants.feederSpeed));

    }

    public void reverseFeeder() {
        feederMotor.setControl(velocityOut.withVelocity(-90.0));
    }

    public void stopFeeder() {
        feederMotor.setControl(dcOut.withOutput(0.0));
        
    }

}
