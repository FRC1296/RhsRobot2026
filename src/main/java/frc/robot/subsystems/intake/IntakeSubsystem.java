package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
 * Deployed position = -0.047119 rotations Undeployed position = 0.312988 rotations
 */
//Johnny the sigma amogus wrote these
/**
 * 1. Use https://www.reca.lc/arm to determine the feedforward constants.
 * 2. Use sysid tool to theoretically determine the PID initial values
 * 3. Mathimatically determine the speed that we want the motor to run, this is our kMaxVelocity
 *      If we want the intake to deploy (move 90degrees) in 1 second... 1/4rps(rev. per second)
 *          this converted to rpm would be 15rpm
 *      If we want the deploy to happen at 15rpm then the deploy motor with a 100/1 gear would need to rotate at 1500rpm
 *      Converting rpm to m/s (https://lucidar.me/en/unit-converter/revolutions-per-second-to-meters-per-second/)
 *          1500rpm, 0.019m(radius of shaft) => 2.9845m/s ~ 3m/s
 * 4. (Phoenix Tuner)TEMP. SET MOTOR TO COAST MODE
 *      we are doing this so we can freely move the system to find encoder positions
 * 5. (Phoenix Tuner)WHAT DIRECTION IS THE INTAKE RUNNING (positive or negative)
 *      Invert motor configuration setting if necessary and check all settings
 * positive goes out negative goes in
 * 6. (Phoenix Tuner)SAME THING WITH THE DEPLOY MOTOR (positive or negative)
 * positive down negative up
 *      Invert motor configuration setting if necessary and check all settings
 * 7. (Phoenix Tuner)ARE WE RUNNING THE INTAKE FAST ENOUGHT TO INTAKE ANYTHING
 *      Adjust intake speed settings as necessary
 * yes
 * 8. (Phoenix Tuner)SET INTAKE MOTOR TO BREAK MODE
 * 9. Comment-out the actual useOutput so motor does not move and 
 *     put the value on the Shuffleboard - verify that values being calculated are valid
 * 10. FIND POSITIONS FOR DEPLOY MOTOR
 *      set all position constants based on these values
 * 11. DETERMINE THE PID VALUESSSSSS
 *      Increase P value until deploy moves as expected with as little overshoot/oscillation as possible
 *      Increase D value until overshoot is decreased with no oscillation
 *      Should not need to tune the I component
 */
public class IntakeSubsystem extends SubsystemBase {

    private TalonFX intakeRollerMotor;
    private TalonFX intakeDeployMotor;
    //private CANcoder intakeAbsEncoder;

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    private VelocityVoltage velocityOut = new VelocityVoltage(0);
    private DutyCycleOut dcOut = new DutyCycleOut(0);

    private DoublePublisher intakePositionPublisher;

    private double intakeDeployPosition = 11.07;
    private double intakeStowPosition = 0;
    private double intakeUndeployPosition = 2.8;
    private double intakeAgitatePosition = 2.8;
    private double intakeAbsDeployPosition = 0.87;
    private double intakeAbsStowPosition = 0.55;


    private double intakeDeploySpeed = 0.10;

    private double deployCruiseVelocity = 75;
    private double deployCruiseAcceleration = 150;
    private double deployCruiseJerk = 0;
    

    private final double deploykP = 9.5;
    private final double deploykI = 0.0;
    private final double deploykD = 0.0;
    private final double deploykG = 0.35;
    private final double deployKA = 0.0;
    private final double deployKS = 0.6;
    private final double deployKV = 0.125;

    private StatusSignal<Boolean> deployMMAtSetpoint;
    private StatusSignal<Voltage> deployVoltage;
    private StatusSignal<Boolean> deployMMEnabled;

    public IntakeSubsystem() {
        super("Intake");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable robotTable = inst.getTable(Constants.NETWORK_TABLE);
        NetworkTable shooterTable = robotTable.getSubTable(Constants.NT_INTAKE);
        intakePositionPublisher = shooterTable.getDoubleTopic(Constants.NT_INTAKE_POSITION).publish();

        intakeRollerMotor = new TalonFX(Constants.intakeConstants.INTAKE_ROLLER_MOTOR_ID);
        intakeDeployMotor = new TalonFX(Constants.intakeConstants.INTAKE_DEPLOY_MOTOR_ID);
        //intakeAbsEncoder = new CANcoder(Constants.intakeConstants.INTAKE_ENCODER_ID);

        //ConfigureAbsoluteEncoder();
        ConfigureIntakeRollerMotor();
        ConfigureDeployMotor();
    }

    // private void ConfigureAbsoluteEncoder() {
    //     CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    //     cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    //     cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    //     cc_cfg.MagnetSensor.withMagnetOffset(Rotations.of(0.0));
    //     intakeAbsEncoder.getConfigurator().apply(cc_cfg);
    // }

    private void ConfigureIntakeRollerMotor() {
        MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(120);
        Slot0Configs slotZeroConfigs = new Slot0Configs()
                .withKP(0.5)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.45)
                .withKV(0.1005);
        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration().withMotorOutput(outputConfig)
                .withCurrentLimits(currentLimitConfig).withSlot0(slotZeroConfigs);

        intakeRollerMotor.getConfigurator().apply(intakeMotorConfig);
    }

    private void ConfigureDeployMotor() {

        MotorOutputConfigs outputConfig = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(55);

        Slot0Configs slotZeroConfigs = new Slot0Configs()
                .withKG(deploykG)
                .withKP(deploykP)
                .withKI(deploykI)
                .withKD(deploykD)
                .withKA(deployKA)
                .withKS(deployKS)
                .withKV(deployKV)
                .withGravityType(GravityTypeValue.Arm_Cosine);

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(deployCruiseVelocity)
                .withMotionMagicAcceleration(deployCruiseAcceleration).withMotionMagicJerk(deployCruiseJerk);

        // FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        //         .withFeedbackRemoteSensorID(Constants.intakeConstants.INTAKE_ENCODER_ID)
        //         .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        //         .withSensorToMechanismRatio(1.0)
        //         .withRotorToSensorRatio(50.0);

        TalonFXConfiguration intakeDeployMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(outputConfig).withCurrentLimits(currentLimitConfig)
                .withSlot0(slotZeroConfigs).withMotionMagic(mmConfigs);
                //.withFeedback(feedbackConfigs);

        intakeDeployMotor.getConfigurator().apply(intakeDeployMotorConfig);
        intakeDeployMotor.setPosition(intakeStowPosition);

        deployMMAtSetpoint = intakeDeployMotor.getMotionMagicAtTarget();
        deployMMEnabled = intakeDeployMotor.getMotionMagicIsRunning();
        deployVoltage = intakeDeployMotor.getMotorVoltage();
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(deployMMAtSetpoint, deployMMEnabled, deployVoltage);
        if (deployMMEnabled.getValue() && deployMMAtSetpoint.getValue()) {
            //resetMotorEncoder();
        }
        //intakePositionPublisher.set(getIntakePosition());
    }

    protected void resetMotorEncoder() {
        //TODO : implement 
    }

    public void runIntake() {
        intakeRollerMotor.setControl(velocityOut.withVelocity(95.0));
    }

    public void stopIntake() {
        intakeRollerMotor.setControl(dcOut.withOutput(0.0));
    }

    public void runIntakeReverse() {
        intakeRollerMotor.setControl(velocityOut.withVelocity(-95.0));
    }

    //public double getIntakePosition() {
        //return intakeAbsEncoder.getAbsolutePosition().getValueAsDouble();
    //}

    public void undeployIntake() {
        intakeDeployMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(intakeUndeployPosition));
    }

    public void stopDeployIntake() {
        intakeDeployMotor.setControl(dcOut.withOutput(0));
    }

    public void deployIntake() {
        intakeRollerMotor.setControl(velocityOut.withSlot(0).withVelocity(95));
        intakeDeployMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(intakeDeployPosition));
    }

    public void moveIntakeToAgitate() {
        intakeDeployMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(intakeAgitatePosition));
    }

     public void manuelUndeployIntake() {
        intakeDeployMotor.setControl(dcOut.withOutput(-intakeDeploySpeed));
    }

     public void manuelDeployIntake() {
        intakeDeployMotor.setControl(dcOut.withOutput(intakeDeploySpeed));
    }

    // public void resetDeployPosition() {
    //     double absPos = intakeAbsEncoder.getAbsolutePosition().getValueAsDouble();
    //     if (absPos >= intakeAbsDeployPosition - .5 && absPos <= intakeAbsDeployPosition + .5) {
    //         intakeDeployMotor.setPosition(intakeDeployPosition);
    //     } else {
    //         if (absPos >= intakeAbsStowPosition - .5 && absPos <= intakeAbsStowPosition + .5) {
    //             intakeDeployMotor.setPosition(intakeStowPosition);
    //         }
    //     }
    // }
}