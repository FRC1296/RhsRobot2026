// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class TurretSubsystem extends SubsystemBase {
    /** Creates a new ShooterSubsystem. */

    private TalonFX turretMotor;
    private DigitalInput hallEffect;

    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    private DutyCycleOut dcOut = new DutyCycleOut(0);

    private DoublePublisher turretPositionPublisher;
    private double cruiseVelocity = 75;

    private final double kP = 7.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kG = 0.0;

    private final double gearRatio = 184.0/10.0;
    private Transform2d turretOffset = new Transform2d(
        new Translation2d(0.0,0.0381),
        new Rotation2d()
    );

    private final double minAngle = -180.0;
    private final double maxAngle = 180.0;

    private CommandSwerveDrivetrain drivetrain;

    public TurretSubsystem(CommandSwerveDrivetrain drive) {

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable robotTable = inst.getTable(Constants.NETWORK_TABLE);
        NetworkTable turretTable = robotTable.getSubTable("Turret Subsystem");
        turretPositionPublisher = turretTable.getDoubleTopic("Turret Position").publish();

        drivetrain = drive;

        turretMotor = new TalonFX(Constants.turretConstants.TURRET_MOTOR_ID);
        hallEffect = new DigitalInput(Constants.turretConstants.HALL_EFFECT_ID);
        
        configureTurretMotor();
    }

    private void configureTurretMotor() {
        MotorOutputConfigs outputConfigs =
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);
        CurrentLimitsConfigs currentConfigs =
                new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(60);
        Slot0Configs slotZeroConfigs = new Slot0Configs().withKG(kG).withKP(kP).withKI(kI).withKD(kD);
        MotionMagicConfigs mmConfigs = new MotionMagicConfigs().withMotionMagicCruiseVelocity(cruiseVelocity)
                .withMotionMagicAcceleration(cruiseVelocity * 2)
                .withMotionMagicJerk(0);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withMotorOutput(outputConfigs)
            .withCurrentLimits(currentConfigs)
            .withSlot0(slotZeroConfigs)
            .withMotionMagic(mmConfigs);

        turretMotor.getConfigurator().apply(motorConfig);
        turretMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        turretPositionPublisher.set(getTurretPosition());
    }

    private double degreesToMotorRotations(double turretDegrees) {
        return (turretDegrees / 360.0) * gearRatio;
    }

    private double motorRotationsToDegrees(double motorRotations) {
        return (motorRotations / gearRatio) * 360.0;
    }

    public void setTurretAngle(double degrees) {
        double aimAngle = MathUtil.inputModulus(getTurretAngle() + degrees, minAngle, maxAngle);
        turretMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(degreesToMotorRotations(aimAngle)));
    }

    public double calculateTurretAngleDelta(Translation2d targetTranslation) {
        Pose2d drivetrainPose = drivetrain.getState().Pose;
        Translation2d turreTranslation = (drivetrainPose.plus(turretOffset)).getTranslation();
        Translation2d vectorToTarget = targetTranslation.minus(turreTranslation);
        double angleToTarget = MathUtil.inputModulus(vectorToTarget.getAngle().getDegrees(), minAngle, maxAngle) - drivetrainPose.getRotation().getDegrees() - getTurretAngle();
        return angleToTarget;
    }

    public void turretAimAt(Translation2d virtualTarget) {
        double desiredAngle = calculateTurretAngleDelta(virtualTarget);
        setTurretAngle(desiredAngle);
    }

    public void turretAimAtHubBool(boolean bool){
        Constants.turretConstants.turretAimAtHub = bool;
    }

    public void turretAimToFeedBool(boolean bool){
        Constants.turretConstants.turretAimToFeed = bool;
    }

    public double getTurretAngle() {
        return motorRotationsToDegrees(turretMotor.getPosition().getValueAsDouble());
    }

    public double getTurretPosition() {
        return turretMotor.getPosition().getValueAsDouble();
    }

    public void moveTurretToZero() {
        turretMotor.setControl(motionMagicVoltage.withSlot(0).withPosition(0.0));
    }

    public void runTurret() {
        turretMotor.setControl(dcOut.withOutput(0.05));
    }

    public void reverseTurret() {
        turretMotor.setControl(dcOut.withOutput(-0.05));
    }

    public void stopTurret() {
        turretMotor.setControl(dcOut.withOutput(0.0));
    }

}
