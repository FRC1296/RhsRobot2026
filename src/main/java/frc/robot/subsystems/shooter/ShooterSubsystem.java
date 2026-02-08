package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX hoodMotor;
    private TalonFX shooterMasterMotor;
    private TalonFX shooterFollowerMotor;

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    private DutyCycleOut dcOut = new DutyCycleOut(0);

    private DoublePublisher hoodPositionPublisher;
    private DoublePublisher shooterSpeedPublisher;

    private double shooterSpeed = 0.85;
    private double hoodPos = 0.0;

    private double hoodCruiseVelocity = 50;
    private final double hoodkP = 10.0;
    private final double hoodkI = 0.0;
    private final double hoodkD = 0.0;
    private final double hoodkG = 0.0;

    public ShooterSubsystem() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable limebotTable = inst.getTable("Robot Data");
        NetworkTable shooterTable = limebotTable.getSubTable("Shooter Subsystem");
        hoodPositionPublisher = shooterTable.getDoubleTopic("Hood Position").publish();
        shooterSpeedPublisher = shooterTable.getDoubleTopic("Shooter Speed").publish();

        hoodMotor = new TalonFX(Constants.shooterConstants.HOOD_MOTOR_ID);
        shooterMasterMotor = new TalonFX(Constants.shooterConstants.SHOOTER_MASTER_MOTOR_ID);
        shooterFollowerMotor = new TalonFX(Constants.shooterConstants.SHOOTER_SLAVE_MOTOR_ID);

        ConfigureHoodMotor();
        ConfigureShooterMotors();
    }

    private void ConfigureHoodMotor() {
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(60);
        Slot0Configs slotZeroConfigs = new Slot0Configs().withKG(hoodkG).withKP(hoodkP).withKI(hoodkI).withKD(hoodkD);
        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicCruiseVelocity(hoodCruiseVelocity)
                .withMotionMagicAcceleration(hoodCruiseVelocity * 2)
                .withMotionMagicJerk(0);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withMotorOutput(outputConfigs)
                .withCurrentLimits(currentConfigs)
                .withSlot0(slotZeroConfigs)
                .withMotionMagic(mmConfigs);

        hoodMotor.getConfigurator().apply(motorConfig);
        hoodMotor.setPosition(0);
    }

    private void ConfigureShooterMotors() {
        shooterFollowerMotor.setControl(
                new Follower(Constants.shooterConstants.SHOOTER_MASTER_MOTOR_ID, MotorAlignmentValue.Opposed));

        MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(110);

        TalonFXConfiguration masterMotorConfig = new TalonFXConfiguration().withCurrentLimits(currentConfigs)
                .withMotorOutput(outputConfig);
        TalonFXConfiguration followerMotorConfig = new TalonFXConfiguration().withCurrentLimits(currentConfigs);

        shooterMasterMotor.getConfigurator().apply(masterMotorConfig);
        shooterFollowerMotor.getConfigurator().apply(followerMotorConfig);

    }

    @Override
    public void periodic() {
        hoodPositionPublisher.set(getHoodPosition());
        shooterSpeedPublisher.set(shooterSpeed);
    }

    public double getHoodPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    public void runMasterShooter() {
        shooterMasterMotor.setControl(dcOut.withOutput(shooterSpeed));
    }

    public void increaseShooterSpeed() {
        if(shooterSpeed <= 0.95){
            shooterSpeed += 0.05;
        }
    }

    public void decreaseShooterSpeed() {
        if(shooterSpeed >= 0.05){
            shooterSpeed -= 0.05;
        }
    }

    public void stopMasterShooter() {
        shooterMasterMotor.setControl(dcOut.withOutput(0.0));
    }

    public void runHood() {
        hoodPos += 0.05;
        hoodMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(hoodPos));
    }

    public void reverseHood() {
        if((hoodPos - 0.05) > 0){
            hoodPos -= 0.05;
            hoodMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(hoodPos));
        }
    }

}
