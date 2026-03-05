//Johnny was here, 03/04/2025, one day before belton competition, we are cooked, they gave me one hour with auton.
//It better works
//updates will be written if everything goes alright
//god bless
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
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autonomous.IAuto;
import frc.robot.autonomous.LeftToDepot;
import frc.robot.autonomous.RightToStation;
import frc.robot.commands.AutoAimAndShootMoving;
import frc.robot.commands.RobotAimAtHub;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.TurretAimToFeed;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.AgitateBalls;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretResetHome;
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
    private DoublePublisher robotDistanceToHubPublisher;

    private SendableChooser<Command> autonChooser = new SendableChooser<>();

    /** 
     * Boolean to tell us if the robot is fully initialized. 
     * On practice field we need to insure that Alliance is set correctly
     */
    private boolean initialized = false;

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

        autoRobotHub = new RobotAimAtHub(this);
        autoAimFeed = new TurretAimToFeed(this);
        autoAaSM = new AutoAimAndShootMoving(this);
        //autoAaS = new AutoAimAndShoot(this);

        configureNetworkTable();
        configureDriverBindings();
        configureOperatorBindings();
        configureAutonOptions();
    }

    private void initialize() {
        if (initialized == false) {
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                hubLocation = Constants.BLUE_HUB;
                feedLocation = Constants.BLUE_FEED_ONE;
            } else {
                hubLocation = Constants.RED_HUB;
                feedLocation = Constants.RED_FEED_ONE;
            }

            autoRobotHub.setTarget(hubLocation);
            autoAimFeed.setFeedLocation(hubLocation);
            autoAaSM.setTarget(hubLocation);
            //autoAaS.setTarget(hubLocation);

            initialized = true;
        }
    }

    private void configureNetworkTable() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable robotTable = inst.getTable(Constants.NETWORK_TABLE);
        haveBallPublisher = robotTable.getBooleanTopic(Constants.NT_INTAKE_HAS_BALL).publish();
        haveBallPublisher.set(false);

        robotVelocityPublisher = robotTable.getDoubleTopic(Constants.NT_ROBOT_VELOCITY).publish();
        robotAnglePublisher = robotTable.getDoubleTopic(Constants.NT_ROBOT_ANGLE).publish(); 

        shooterVelocityPublisher = robotTable.getDoubleTopic(Constants.NT_SHOOTER_VELOCITY).publish();
        turretAnglePublisher = robotTable.getDoubleTopic(Constants.NT_TURRET_ANGLE).publish();

        robotDistanceToHubPublisher = robotTable.getDoubleTopic(Constants.NT_ROBOT_DISTANCE_TO_HUB).publish();
    }

    private void configureOperatorBindings() {
        operatorJoystick.rightTrigger().toggleOnTrue(shootBalls);
        operatorJoystick.rightBumper().whileTrue(new InstantCommand(spindexer::runSpindexer)).onFalse(new InstantCommand(spindexer::stopSpindexer));
        operatorJoystick.rightBumper().whileTrue(new InstantCommand(feeder::runFeeder)).onFalse(new InstantCommand(feeder::stopFeeder));
        operatorJoystick.leftBumper().whileTrue(new ParallelCommandGroup(
            new InstantCommand(spindexer::reverseSpindexer),
            new InstantCommand(feeder::reverseFeeder)
        ));

        operatorJoystick.leftTrigger().onTrue(autoAaSM);
        //operatorJoystick.leftBumper().onTrue(autoAimFeed);
        operatorJoystick.x().onTrue(new InstantCommand(() -> turret.turretAimAtHubBool(false)));
        //operatorJoystick.a().onTrue(new InstantCommand(shooter::fullCourtShoot));

        //operatorJoystick.b().onTrue(new InstantCommand(() -> turret.turretAimToFeedBool(false)));
        // operatorJoystick.b().onTrue(new InstantCommand(shooter::moveHoodUp));
         //operatorJoystick.y().onTrue(new AgitateBalls(intake));
        operatorJoystick.a().onTrue(new InstantCommand(shooter::decreaseShooterSpeed));
        operatorJoystick.y().onTrue(new InstantCommand(shooter::increaseShooterSpeed));
        
       
        operatorJoystick.povLeft().onTrue(new InstantCommand(intake::undeployIntake));
        operatorJoystick.povRight().onTrue(new InstantCommand(intake::deployIntake));

        operatorJoystick.back().onTrue(autoRobotHub);
        operatorJoystick.start().onTrue(new InstantCommand(() -> LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b")));
    }

    private void configureDriverBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate)));
        driverJoystick.a().onTrue(new TurretResetHome(this).andThen(new InstantCommand(turret::resetTurretZero)));
        driverJoystick.x().whileTrue(new InstantCommand(turret::runTurret)).whileFalse(new InstantCommand(turret::stopTurret));
        driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> brake));

        // driverJoystick.leftTrigger().onTrue(new
        // InstantCommand(intake::deployIntake));
        // driverJoystick.leftBumper().onTrue(new
        // InstantCommand(intake::undeployIntake));
        driverJoystick.rightTrigger().whileTrue(new InstantCommand(intake::runIntake)).whileFalse(new InstantCommand(intake::stopIntake));

        driverJoystick.povLeft().onTrue(new InstantCommand(turret::decreaseTurretAngle));
        driverJoystick.povRight().onTrue(new InstantCommand(turret::increaseTurretAngle));
        driverJoystick.povUp().onTrue(new InstantCommand(shooter::increaseShooterInterpSpeed));
        driverJoystick.povDown().onTrue(new InstantCommand(shooter::decreaseShooterInterpSpeed));

        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        drivetrain.registerTelemetry(logger::telemeterize);

        //driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new
        // Rotation2d(-joystick.getLeftY(),-joystick.getLeftX()))));
    }

    public void configureAutonOptions() {
        SendableRegistry.setName(autonChooser, "Autonomous Options");
        boolean redPath = true;
        boolean bluePath = false;

        autonChooser.addOption("RightToStation Red", new RightToStation(this, MaxSpeed, MaxAngularRate, redPath));
        autonChooser.addOption("RightToStation Blue", new RightToStation(this, MaxSpeed, MaxAngularRate, bluePath));

        autonChooser.addOption("LeftToDepot Red", new LeftToDepot(this, MaxSpeed, MaxAngularRate, redPath));
        autonChooser.addOption("LeftToDepot Blue", new LeftToDepot(this, MaxSpeed, MaxAngularRate, bluePath));

        SmartDashboard.putData("Auto Choices", autonChooser);
    }

    public void autonomousInit() {
        initialize();
    }

    public Command getAutonomousCommand() {
        Command auton = autonChooser.getSelected();

        //drivetrain.resetPose(((IAuto) auton).getInitialPose());
        if (LocalizationHelpers.tagInVison("limelight-a") || LocalizationHelpers.tagInVison("limelight-b")) {
            LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b");
        } else {
            drivetrain.resetPose(((IAuto) auton).getInitialPose());
        }

        shooter.setDefaultCommand(new AutoAimAndShootMoving(this, hubLocation.getX(), hubLocation.getY()));
        return auton;
    }

    public void autonomousExit() {
        turret.turretAimAtHubBool(false);
    }

    public void robotPeriodic() {
        LocalizationHelpers.updatePose(drivetrain, "limelight-a");
        LocalizationHelpers.updatePose(drivetrain, "limelight-b");

        //getDriveVector();
        robotVelocityPublisher.set(robotVelocity);
        robotAnglePublisher.set(robotAngle);

        shooterVelocityPublisher.set(shooter.getShooterVelocity());
        turretAnglePublisher.set(turret.getTurretAngle());

        //Publish the robot distance to the hub
        if (hubLocation != null) {
            Translation2d robotLocation = drivetrain.getPose().plus(shooter.getShooterOffset()).getTranslation();
            robotDistanceToHubPublisher.set(robotLocation.getDistance(hubLocation));
        }
    }

    public void teleopInit() {
        initialize();
        // Command auton = new LeftToDepot(this, MaxSpeed, MaxAngularRate, true);
        // drivetrain.resetPose(((IAuto) auton).getInitialPose());

    }

    public void teleopPeriodic() {
        // if (drivetrain.getState().Pose.getX() < 4.6 || drivetrain.getState().Pose.getX() > 11.9) {
        //     LimelightHelpers.SetThrottle("limelight-a", 0);
        //     LimelightHelpers.SetThrottle("limelight-b", 0);
        // } else {
        //     LimelightHelpers.SetThrottle("limelight-a", 50);
        //     LimelightHelpers.SetThrottle("limelight-b", 50);
        // }

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
    //     robotVelocity = Math.hypot(vx, vy);

    //     // Direction (Angle): theta = inverse tan(vy/vx)
    //     robotAngle = Math.toDegrees(Math.atan2(vy, vx));

    //     Translation2d test = new Translation2d(robotVelocity,robotAngle);
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
