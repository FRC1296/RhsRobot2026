package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

        private TalonFX intakeRollerMotor;
        private TalonFX intakeDeployMotor;

        private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
        private DutyCycleOut dcOut = new DutyCycleOut(0);

        private DoublePublisher intakePositionPublisher;

        private double deployCruiseVelocity = 50;

        private final double deploykP = 10.0;
        private final double deploykI = 0.0;
        private final double deploykD = 0.0;
        private final double deploykG = 0.0;

        public IntakeSubsystem() {
                NetworkTableInstance inst = NetworkTableInstance.getDefault();
                NetworkTable limebotTable = inst.getTable("Robot Data");
                NetworkTable shooterTable = limebotTable.getSubTable("Intake Subsystem");
                intakePositionPublisher = shooterTable.getDoubleTopic("Deploy Position").publish();

                intakeRollerMotor = new TalonFX(Constants.intakeConstants.INTAKE_ROLLER_MOTOR_ID);
                intakeDeployMotor = new TalonFX(Constants.intakeConstants.INTAKE_DEPLOY_MOTOR_ID);

                ConfigureIntakeRollerMotor();
                ConfigureDeployMotor();
        }

        private void ConfigureIntakeRollerMotor() {

                MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                                .withInverted(InvertedValue.Clockwise_Positive);
                CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(80);

                TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration()
                                .withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig);

                intakeRollerMotor.getConfigurator().apply(intakeMotorConfig);
        }

        private void ConfigureDeployMotor() {

                MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                                .withInverted(InvertedValue.CounterClockwise_Positive);
                CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(90);
                Slot0Configs slotZeroConfigs = new Slot0Configs().withKG(deploykG).withKP(deploykP).withKI(deploykI)
                                .withKD(deploykD);
                MotionMagicConfigs mmConfigs = new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(deployCruiseVelocity)
                                .withMotionMagicAcceleration(deployCruiseVelocity * 2)
                                .withMotionMagicJerk(0);

                TalonFXConfiguration intakeDeployMotorConfig = new TalonFXConfiguration()
                                .withMotorOutput(outputConfig)
                                .withCurrentLimits(currentLimitConfig)
                                .withSlot0(slotZeroConfigs)
                                .withMotionMagic(mmConfigs);

                intakeDeployMotor.getConfigurator().apply(intakeDeployMotorConfig);
                intakeDeployMotor.setPosition(0);
        }

        public void periodic() {
                intakePositionPublisher.set(getIntakePosition());
        }

        public void runIntake() {
                intakeRollerMotor.setControl(dcOut.withOutput(0.50));
        }

        public void stopIntake() {
                intakeRollerMotor.setControl(dcOut.withOutput(0.0));
        }

        public double getIntakePosition() {
                return intakeDeployMotor.getPosition().getValueAsDouble();
        }

        public void undeployIntake() {
                intakeDeployMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(-1.0));
        }
}
