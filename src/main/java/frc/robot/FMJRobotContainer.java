package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autonomous.DestoryerHistoryMakingTestAuton;
import frc.robot.autonomous.IAuto;
import frc.robot.autonomous.ShootAuton;
import frc.robot.commands.AutoAimAndShoot;
import frc.robot.commands.RobotAimAtHub;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.TurretAimToFeed;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.LocalizationHelpers;

public class FMJRobotContainer {

  private double MaxSpeed = Constants.driveConstants.driveSpeed;
  private double MaxAngularRate = Constants.driveConstants.rotationSpeed;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 15% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new
  // SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driverJoystick = new CommandXboxController(0);
  private final CommandXboxController operatorJoystick = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private ShooterSubsystem shooter = new ShooterSubsystem(drivetrain);
  private TurretSubsystem turret = new TurretSubsystem(drivetrain);
  private IntakeSubsystem intake = new IntakeSubsystem();
  private FeederSubsystem feeder = new FeederSubsystem();
  private ClimberSubsystem climber = new ClimberSubsystem();
  private SpindexerSubsystem spindexer = new SpindexerSubsystem();
  private LedSubsystem LED = new LedSubsystem();

  private AutoAimAndShoot autoAaS;
  private RobotAimAtHub autoRobotHub;
  private TurretAimToFeed autoAimFeed;
  private ShootBalls shootBalls = new ShootBalls(this);

  public FMJRobotContainer() {

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      autoRobotHub = new RobotAimAtHub(this, 4.6, 4);
    } else {
      autoRobotHub = new RobotAimAtHub(this, 11.9, 4);
    }
    autoAimFeed = new TurretAimToFeed(this, 0.0, 0.0);
    autoAaS = new AutoAimAndShoot(this);

    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureOperatorBindings() {
    operatorJoystick.rightTrigger().toggleOnTrue(shootBalls);
    operatorJoystick.rightBumper().toggleOnTrue(Commands.startEnd(spindexer::runSpindexer,spindexer::stopSpindexer,spindexer));
    operatorJoystick.leftTrigger().onTrue(autoAaS);
    operatorJoystick.leftBumper().onTrue(autoAimFeed);
    operatorJoystick.x().onTrue(new InstantCommand(() -> turret.turretAimAtHubBool(false)));
    operatorJoystick.x().onTrue(new InstantCommand(() -> shooter.shooterAutoInterpolateBool(false)));
    operatorJoystick.b().onTrue(new InstantCommand(() -> turret.turretAimToFeedBool(false)));
    // operatorJoystick.y().onTrue(new
    // InstantCommand(shooter::increaseShooterSpeed));
    // operatorJoystick.a().onTrue(new
    // InstantCommand(shooter::decreaseShooterSpeed));

    // operatorJoystick.povRight().whileTrue(new
    // InstantCommand(turret::runTurret)).onFalse(new
    // InstantCommand(turret::stopTurret));
    // operatorJoystick.povLeft().whileTrue(new
    // InstantCommand(turret::reverseTurret)).onFalse(new
    // InstantCommand(turret::stopTurret));
    // operatorJoystick.povUp().onTrue(new InstantCommand(shooter::runHood));
    // operatorJoystick.povDown().onTrue(new InstantCommand(shooter::reverseHood));

    operatorJoystick.back().onTrue(autoRobotHub);
    operatorJoystick.start().onTrue(
        new InstantCommand(() -> LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b")));
  }

  private void configureDriverBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
            .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
            .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate)));

    driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> brake));

    // driverJoystick.leftTrigger().onTrue(new
    // InstantCommand(intake::deployIntake));
    // driverJoystick.leftBumper().onTrue(new
    // InstantCommand(intake::undeployIntake));
    driverJoystick.rightTrigger().whileTrue(new InstantCommand(intake::runIntake))
        .whileFalse(new InstantCommand(intake::stopIntake));
    // driverJoystick.y().onTrue(new InstantCommand(shooter::moveToPositionOne));
    // driverJoystick.a().onTrue(new InstantCommand(shooter::moveToZero));

    driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    drivetrain.registerTelemetry(logger::telemeterize);

    driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    // joystick.b().whileTrue(drivetrain.applyRequest(() ->
    // point.withModuleDirection(new
    // Rotation2d(-joystick.getLeftY(),-joystick.getLeftX()))));
  }

  public Command getAutonomousCommand() {
    Command auton = new ShootAuton(this, MaxSpeed, MaxAngularRate, false);

    Pose2d initialPose;
    if (auton instanceof IAuto) {
      initialPose = ((IAuto) auton).getInitialPose();
    } else {
      initialPose = new Pose2d();
    }
    drivetrain.initializePoseForAutonomous(initialPose);

    SmartDashboard.putString("Auton Pose", "x-" + initialPose.getX() + ", y-" + initialPose.getY() + ", rotation-"
        + initialPose.getRotation().getDegrees());

    return auton;
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

    LocalizationHelpers.updateFieldPosition(drivetrain, "limelight-a");
    // LocalizationHelpers.updateFieldPosition(drivetrain, "limelight-b");

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (drivetrain.getPose().getY() > 4.0) {
        autoAimFeed = new TurretAimToFeed(this, 2.0, 6.5);
      } else {
        autoAimFeed = new TurretAimToFeed(this, 2.0, 1.5);
      }
    } else {
      if (drivetrain.getPose().getY() > 4.0) {
        autoAimFeed = new TurretAimToFeed(this, 14.5, 6.5);
      } else {
        autoAimFeed = new TurretAimToFeed(this, 14.5, 1.5);
      }
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

  public FeederSubsystem getFeeder() {
    return feeder;
  }

  public SpindexerSubsystem getSpindexer() {
    return spindexer;
  }
  public ShooterSubsystem getShooter() {
    return shooter;
  }

  public IntakeSubsystem getIntake() {
    return intake;
  }

  public ClimberSubsystem getClimber() {
    return climber;
  }

}
