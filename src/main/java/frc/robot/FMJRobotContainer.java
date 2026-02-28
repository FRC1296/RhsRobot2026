package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autonomous.IAuto;
import frc.robot.autonomous.LeftToDepot;
import frc.robot.autonomous.RightToStation;
import frc.robot.autonomous.TestAuton;
import frc.robot.commands.AutoAimAndShoot;
import frc.robot.commands.AutoAimAndShootMoving;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.RobotAimAtHub;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.TurretAimAtHub;
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

    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private FeederSubsystem feeder;
    private ClimberSubsystem climber;
    private SpindexerSubsystem spindexer;
    private LedSubsystem LED;

    //private AutoAimAndShoot autoAaS;
    private AutoAimAndShootMoving autoAaSM;
    private RobotAimAtHub autoRobotHub;
    private TurretAimToFeed autoAimFeed;
    private ShootBalls shootBalls;

    private BooleanPublisher haveBallPublisher;

    private Translation2d hubLocation;
    private Translation2d feedLocation;

    private DoublePublisher robotVelocityPublisher;
    private DoublePublisher robotAnglePublisher;
    private double robotVelocity;
    private double robotAngle;

    private DoublePublisher shooterVelocityPublisher;
    private DoublePublisher turretAnglePublisher;

    public FMJRobotContainer() {

        drivetrain.getPigeon2().setYaw(0.0);

        shooter = new ShooterSubsystem(drivetrain);
        turret = new TurretSubsystem(drivetrain);
        intake = new IntakeSubsystem();
        feeder = new FeederSubsystem();
        climber = new ClimberSubsystem();
        spindexer = new SpindexerSubsystem();
        LED = new LedSubsystem();

        shootBalls = new ShootBalls(this);

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            hubLocation = Constants.BLUE_HUB;
            feedLocation = Constants.BLUE_FEED_ONE;
        } else {
            hubLocation = Constants.RED_HUB;
            feedLocation = Constants.RED_FEED_ONE;
        }

        autoRobotHub = new RobotAimAtHub(this, hubLocation.getX(), hubLocation.getY());
        autoAimFeed = new TurretAimToFeed(this, feedLocation.getX(), feedLocation.getY());
        autoAaSM = new AutoAimAndShootMoving(this, hubLocation.getX(), hubLocation.getY());
        //autoAaS = new AutoAimAndShoot(this);


        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable robotTable = inst.getTable(Constants.NETWORK_TABLE);
        haveBallPublisher = robotTable.getBooleanTopic(Constants.NT_INTAKE_HAS_BALL).publish();
        haveBallPublisher.set(false);

        robotVelocityPublisher = robotTable.getDoubleTopic(Constants.NT_ROBOT_VELOCITY).publish();
        robotAnglePublisher = robotTable.getDoubleTopic(Constants.NT_ROBOT_ANGLE).publish(); 

        shooterVelocityPublisher = robotTable.getDoubleTopic(Constants.NT_SHOOTER_VELOCITY).publish();
        turretAnglePublisher = robotTable.getDoubleTopic(Constants.NT_TURRET_ANGLE).publish();

        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureOperatorBindings() {
        operatorJoystick.rightTrigger().toggleOnTrue(shootBalls);
        operatorJoystick.rightBumper().whileTrue(new InstantCommand(spindexer::runSpindexer)).onFalse(new InstantCommand(spindexer::stopSpindexer));
        operatorJoystick.leftTrigger().onTrue(autoAaSM);
        //operatorJoystick.leftBumper().onTrue(autoAimFeed);
        operatorJoystick.x().onTrue(new InstantCommand(() -> turret.turretAimAtHubBool(false)));
        //operatorJoystick.b().onTrue(new InstantCommand(() -> turret.turretAimToFeedBool(false)));
        operatorJoystick.y().onTrue(new InstantCommand(shooter::increaseShooterSpeed));
        operatorJoystick.a().onTrue(new InstantCommand(shooter::decreaseShooterSpeed));

        // operatorJoystick.povRight().whileTrue(new
        // InstantCommand(turret::runTurret)).onFalse(new
        // InstantCommand(turret::stopTurret));
        // operatorJoystick.povLeft().whileTrue(new
        // InstantCommand(turret::reverseTurret)).onFalse(new
        // InstantCommand(turret::stopTurret));
        operatorJoystick.povUp().onTrue(new InstantCommand(shooter::runHood));
        operatorJoystick.povDown().onTrue(new InstantCommand(shooter::reverseHood));

        operatorJoystick.back().onTrue(autoRobotHub);
        operatorJoystick.start().onTrue(new InstantCommand(() -> LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b")));
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
        driverJoystick.rightTrigger().whileTrue(new InstantCommand(intake::runIntake)).whileFalse(new InstantCommand(intake::stopIntake));
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

    // is on startup
    public Command getAutonomousCommand() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Command auton = new RightToStation(this, MaxSpeed, MaxAngularRate, isRed);

        if (LocalizationHelpers.tagInVison("limelight-a")) {
            LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b");
        } else {
            drivetrain.resetPose(((IAuto) auton).getInitialPose());
        }

        shooter.setDefaultCommand(new AutoAimAndShootMoving(this, hubLocation.getX(), hubLocation.getY()));
        return auton;
    }

    public void autonomousExit() {
        shooter.setDefaultCommand(null);
        turret.setDefaultCommand(null);
    }

    public void robotPeriodic() {
        LocalizationHelpers.updatePose(drivetrain, "limelight-a");
        LocalizationHelpers.updatePose(drivetrain, "limelight-b");

        // getDriveVector();
        robotVelocityPublisher.set(robotVelocity);
        robotAnglePublisher.set(robotAngle);

        shooterVelocityPublisher.set(shooter.getShooterVelocity());
        turretAnglePublisher.set(turret.getTurretAngle());

    }

    public void teleopPeriodic() {
        if (drivetrain.getState().Pose.getX() < 4.6 || drivetrain.getState().Pose.getX() > 11.9) {
            LimelightHelpers.SetThrottle("limelight-a", 0);
            LimelightHelpers.SetThrottle("limelight-b", 0);
        } else {
            LimelightHelpers.SetThrottle("limelight-a", 50);
            LimelightHelpers.SetThrottle("limelight-b", 50);
        }

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            if (drivetrain.getPose().getY() > 4.0) {
                autoAimFeed.setFeedLocation(Constants.BLUE_FEED_ONE);
            } else {
                autoAimFeed.setFeedLocation(Constants.BLUE_FEED_TWO);
            }
        } else {
            if (drivetrain.getPose().getY() > 4.0) {
                autoAimFeed.setFeedLocation(Constants.RED_FEED_ONE);
            } else {
                autoAimFeed.setFeedLocation(Constants.RED_FEED_TWO);
            }
        }
    }

    public void setInitialPose(double x, double y) {
        drivetrain.resetPose(new Pose2d(x, y, drivetrain.getState().Pose.getRotation()));
    }

    // public Vector<N2> getDriveVector() {
    //     ChassisSpeeds cs = drivetrain.getKinematics().toChassisSpeeds();
    //     double vx = cs.vxMetersPerSecond;
    //     double vy = cs.vyMetersPerSecond;
    
    //     // Total Speed (Magnitude): V = sqrt(vx2 + vy2)
    //     double totalV = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
    //     robotVelocity = totalV;

    //     // Direction (Angle): theta = inverse tan(vy/vx)
    //     double theta = Math.toDegrees(Math.atan2(vy, vx));
    //     robotAngle = theta;

    //     Translation2d test = new Translation2d(totalV,theta);
    //     return test.toVector();
       
    // }

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

    public Translation2d getHubLocation() {
        return hubLocation;
    }
}
