// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class IntakePIDSubsystem extends ProfiledPIDSubsystem {
    private TalonFX intakeDeployMotor;
    private CANcoder intakeAbsEncoder;

    // PID Controller constants
    // Calculated using sysid tool
    // Feedforward Gains (Theoretical)
    // Kv = 1.88
    // Ka = 0.01
    // Response Timescale = 5.3191
    // Feedback Analysis
    // Gain Preset: CRE (Pro)
    // Max Controller Output: 12.0
    // Velocity Denomintator Units: 0.1
    // Controller Period: 1
    // Time-normalized: true
    // Measurement delay: 20
    // Loop Type: Position
    // Kp = 5.9887
    // Kd = 2.4394
    // Max Position Error: 1
    // Max Velocity Error: 1.5
    // Max Control Effort: 7

    private static final double kP = 12.5;
    private static final double kI = 4.5;
    private static final double kD = 3.5;

    // Feed Forward Constants
    // Calculated using https://www.reca.lc/arm
    // Motor: 1 Falcon 500 (FOFC)
    // Ratio: 100 [Reduction]
    // Efficiency(%): 87
    // Current Limit: 30 (A) [this is the stator current]
    // CoM Distance: 9 (inches)
    // Arm Mass: 4 (lbs)
    // Start Angle: 0 (deg)
    // End Angle: 90 (deg)
    // Iteration Limit: 10000
    private final int currentLimit = 100;
    private final double kG = 0.07;
    private final double kV = 1.88;
    private final double kS = 0.0; // Not generated from above website so set to zero

    private static final double kMaxVelocity = 3.0; // 30
    private static final double kMaxAcceleration = 3.0; // 30
    private static TrapezoidProfile.Constraints profileConstraints = 
        new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);

    private static ProfiledPIDController pidController = 
        new ProfiledPIDController(kP, kI, kD, profileConstraints);

    private ArmFeedforward feedForword;

    protected static double START_POSITION = 0.0;

    public IntakePIDSubsystem() {
        super(pidController, START_POSITION);
        pidController.setTolerance(0.1);
        pidController.setIZone(.05);

        intakeDeployMotor = new TalonFX(Constants.intakeConstants.INTAKE_DEPLOY_MOTOR_ID);
        intakeAbsEncoder = new CANcoder(Constants.intakeConstants.INTAKE_ENCODER_ID);

        ConfigureAbsoluteEncoder();
        ConfigureDeployMotor();

        feedForword = new ArmFeedforward(kS, kG, kV);

        setGoal(START_POSITION);
    }

    private void ConfigureAbsoluteEncoder() {
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cc_cfg.MagnetSensor.withMagnetOffset(Rotations.of(0.0));
        intakeAbsEncoder.getConfigurator().apply(cc_cfg);
    }

    private void ConfigureDeployMotor() {
        MotorOutputConfigs outputConfig = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(120);

        TalonFXConfiguration intakeDeployMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(outputConfig)
                .withCurrentLimits(currentLimitConfig);

        intakeDeployMotor.getConfigurator().apply(intakeDeployMotorConfig);
        intakeDeployMotor.setPosition(0.0);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double setpointRadians = convertPositonToRadians(setpoint.position);
        double feedforward = feedForword.calculate(setpointRadians, setpoint.velocity);

        intakeDeployMotor.setVoltage(output + feedforward);
    }

    @Override
    protected double getMeasurement() {
        return intakeAbsEncoder.getAbsolutePosition().getValueAsDouble();
    }

    protected double convertPositonToRadians(double position) {
        // Absolute Encoder Intake down - 
        // Absolute Encoder Intake 90 deg up -
        // TODO: implement a conversion of absolute encoder value to a radian value
        // double oldMin = 0.95;
        // double oldMax = 0.2;
        // double oldRange = -0.75;
        // double newMin = -90; // -90degrees
        // double newMax = 180; // 180degrees
        // double newRange = 270;
        // if (position > oldMin) {
        //     position = oldMin;
        // } else if (position < oldMax) {
        //     position = oldMax;
        // }
        // double result = (((position - oldMin) / oldRange) * 270) + newMin;

        double result = 0.0;
        return Units.degreesToRadians(result);
    }
}
