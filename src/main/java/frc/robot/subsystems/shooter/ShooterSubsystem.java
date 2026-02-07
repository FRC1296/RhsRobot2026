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
        private TalonFX shooterFeederMotor;

        private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
        private DutyCycleOut dcOut = new DutyCycleOut(0);

        private DoublePublisher hoodPositionPublisher;

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

                hoodMotor = new TalonFX(Constants.shooterConstants.HOOD_MOTOR_ID);
                shooterMasterMotor = new TalonFX(Constants.shooterConstants.SHOOTER_MASTER_MOTOR_ID);
                shooterFollowerMotor = new TalonFX(Constants.shooterConstants.SHOOTER_SLAVE_MOTOR_ID);
                shooterFeederMotor = new TalonFX(Constants.shooterConstants.SHOOTER_FEEDER_MOTOR_ID);

                ConfigureHoodMotor();
                ConfigureShooterMasterMotor();
                ConfigureShooterFeederMotor();
        }

        private void ConfigureHoodMotor() {
                MotorOutputConfigs outputConfigs = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
                CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(60);
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

        private void ConfigureShooterMasterMotor() {
                shooterFollowerMotor.setControl(new Follower(shooterMasterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

                CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(110);

                TalonFXConfiguration motorConfig = new TalonFXConfiguration().withCurrentLimits(currentConfigs);
                shooterMasterMotor.getConfigurator().apply(motorConfig);
                shooterFollowerMotor.getConfigurator().apply(motorConfig);

        }

        private void ConfigureShooterFeederMotor() {
                MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.Clockwise_Positive);
                CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(70);

                TalonFXConfiguration shooterFeederMotorConfig = new TalonFXConfiguration().withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig);

                shooterFeederMotor.getConfigurator().apply(shooterFeederMotorConfig);
        }

    @Override
    public void periodic() {
        hoodPositionPublisher.set(getHoodPosition());
    }

    public double getHoodPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
    } 

    public void moveHoodToMax() {
        hoodMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(-1.0));
    }

    public void moveHoodToMin() {
        hoodMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(-0.05));
    }

    public void runMasterShooter() {
        shooterMasterMotor.setControl(dcOut.withOutput(0.90));
    }

    public void stopMasterShooter() {
        shooterMasterMotor.setControl(dcOut.withOutput(0.0));
    }

}
