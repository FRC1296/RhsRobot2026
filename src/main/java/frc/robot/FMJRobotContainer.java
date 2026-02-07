// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.RobotAimAtHub;
import frc.robot.commands.TurretAimAtHub;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.LocalizationHelpers;

public class FMJRobotContainer {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.25;
  private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  // private final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new
  // SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driverJoystick = new CommandXboxController(0);
  private final CommandXboxController operatorJoystick = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private ShooterSubsystem shooter = new ShooterSubsystem();
  private TurretSubsystem turret = new TurretSubsystem(drivetrain);
  private LedSubsystem LED = new LedSubsystem();

  private TurretAimAtHub autoTurretHub;
  private RobotAimAtHub autoRobotHub;

  public FMJRobotContainer() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      autoTurretHub = new TurretAimAtHub(this, 4.6, 4);
      autoRobotHub = new RobotAimAtHub(this, 4.6, 4);
    } else {
      autoTurretHub = new TurretAimAtHub(this, 11.9, 4);
      autoRobotHub = new RobotAimAtHub(this, 11.9, 4);
    }
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureOperatorBindings() {
    operatorJoystick.rightTrigger().whileTrue(new InstantCommand(shooter::runMasterShooter))
        .whileFalse(new InstantCommand(shooter::stopMasterShooter));
    operatorJoystick.x().onTrue(autoTurretHub);
    operatorJoystick.y().onTrue(new InstantCommand(() -> turret.turretAimAtHubBool(false)));
    operatorJoystick.a().onTrue(autoRobotHub);
  }

  private void configureDriverBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),-joystick.getLeftX()))));

    driverJoystick.y().onTrue(new InstantCommand(() -> LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b")));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    // driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() ->
    // drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return new PrintCommand("No Auto Loaded");
  }

  public void robotPeriodic() {
  }

  public void teleopPeriodic() {
    if (drivetrain.getState().Pose.getX() < 4.6 || drivetrain.getState().Pose.getX() > 11.9) {
      LimelightHelpers.SetThrottle("limelight-a", 0);
      LimelightHelpers.SetThrottle("limelight-b", 0);
    } else {
      LimelightHelpers.SetThrottle("limelight-a", 50);
      LimelightHelpers.SetThrottle("limelight-b", 50);
    }
  }

  public void setInitialPose(double x, double y) {
    drivetrain.resetPose(new Pose2d(x, y, drivetrain.getState().Pose.getRotation()));
  }

  public TurretSubsystem getTurret() {
    return turret;
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

}
