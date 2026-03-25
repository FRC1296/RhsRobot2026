package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.FullMidSwipe;
import frc.robot.autonomous.IAuto;
import frc.robot.autonomous.LeftMidShoot;
import frc.robot.autonomous.LeftMidShootDepot;
import frc.robot.autonomous.LeftToDepot;
import frc.robot.autonomous.LeftToMiddle;
import frc.robot.autonomous.RightToStation;
import frc.robot.autonomous.ShootAuton;
import frc.robot.commands.AutoAimAndShootMoving;
import frc.robot.commands.FeedAimAndShootMoving;
import frc.robot.commands.RobotAimAtHub;
import frc.robot.commands.ShootBalls;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.AgitateBalls;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.Localization;


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
    private FeedAimAndShootMoving autoAimFeed;
    private RobotAimAtHub autoRobotHub;
    private ShootBalls shootBalls;
    private AgitateBalls agitate;

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
            drivetrain.getPigeon2().setYaw(0.0);
        } else {
            hubLocation = Constants.RED_HUB;
            feedLocation = Constants.RED_FEED_ONE;
            drivetrain.getPigeon2().setYaw(180.0);
        }

        autoRobotHub = new RobotAimAtHub(this);
        autoAimFeed = new FeedAimAndShootMoving(this);
        autoAaSM = new AutoAimAndShootMoving(this);
        //autoAaS = new AutoAimAndShoot(this);

        NamedCommands.registerCommand("runIntake", new InstantCommand(intake::runIntake));
        NamedCommands.registerCommand("resetLLP", new InstantCommand(() -> Localization.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b")));
        NamedCommands.registerCommand("runFeeder", new InstantCommand(feeder::runFeeder, feeder));
        NamedCommands.registerCommand("runSpindexer", new InstantCommand(spindexer::runSpindexer));
        NamedCommands.registerCommand("stopSpindexer", new InstantCommand(spindexer::stopSpindexer));
        NamedCommands.registerCommand("stopIntake", new InstantCommand(intake::stopIntake));
        NamedCommands.registerCommand("stopFeeder", new InstantCommand(feeder::stopFeeder));
        NamedCommands.registerCommand("deployIntake", new InstantCommand(intake::deployIntake));
        NamedCommands.registerCommand("undeployIntake", new InstantCommand(intake::undeployIntake));

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
            autoAimFeed.setTarget(feedLocation);
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

    private void configureOperatorBindings() {;

        operatorJoystick.leftTrigger().toggleOnTrue(shootBalls);
        operatorJoystick.leftBumper().whileTrue(new ParallelCommandGroup(
            new InstantCommand(spindexer::reverseSpindexer),
            new InstantCommand(feeder::reverseFeeder)
        )).onFalse(new ParallelCommandGroup(
            new InstantCommand(spindexer::stopSpindexer),
            new InstantCommand(feeder::stopFeeder)
        ));
        operatorJoystick.rightTrigger().whileTrue(new InstantCommand(intake::manuelDeployIntake)).onFalse(new InstantCommand(intake::stopDeployIntake));
        operatorJoystick.rightBumper().whileTrue(new InstantCommand(spindexer::runSpindexer))
        .onFalse(new InstantCommand(spindexer::stopSpindexer));
        operatorJoystick.rightBumper().whileTrue(new InstantCommand(feeder::runFeeder))
        .onFalse(new InstantCommand(feeder::runFeeder).withTimeout(1.0).andThen(new InstantCommand(feeder::stopFeeder)));

        operatorJoystick.x().onTrue(new InstantCommand(() -> turret.turretAimAtHubBool(false)));
        operatorJoystick.b().onTrue(new InstantCommand(() -> turret.turretAimToFeedBool(false)));
        //operatorJoystick.y().onTrue(new InstantCommand(shooter::increaseToF));
        //operatorJoystick.a().onTrue(new InstantCommand(shooter::decreaseToF));

        operatorJoystick.povRight().onTrue(new InstantCommand(turret::increaseTurretAngle));
        operatorJoystick.povLeft().onTrue(new InstantCommand(turret::decreaseTurretAngle));
        //operatorJoystick.povUp().onTrue(new InstantCommand(shooter::increaseShooterInterpSpeed));
        //operatorJoystick.povDown().onTrue(new InstantCommand(shooter::decreaseShooterInterpSpeed));

        operatorJoystick.start().onTrue(new InstantCommand(() -> Localization.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b")));
        //operatorJoystick.back().onTrue(new InstantCommand(intake::resetDeployPosition));

    }

    private void configureDriverBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate)));
        driverJoystick.y().whileTrue(drivetrain.applyRequest(() -> drive
            .withVelocityX(-driverJoystick.getLeftY() * MaxSpeed * 0.5)
            .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed * 0.5)
            .withRotationalRate(-driverJoystick.getRightX() * 2  )));
        // driverJoystick.rightStick().whileTrue(
        //     drivetrain.applyRequest(() -> drive
        //         .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate * 0.5  )
        //         .withCenterOfRotation(hubLocation)
        //     )
        // );

        //driverJoystick.x().onTrue(new TurretResetHome(this).andThen(new InstantCommand(turret::resetTurretZero)));

        driverJoystick.rightTrigger().whileTrue(new InstantCommand(intake::runIntake)).onFalse(new InstantCommand(intake::stopIntake));
        driverJoystick.rightBumper().whileTrue(new InstantCommand(intake::runIntakeReverse)).onFalse(new InstantCommand(intake::stopIntake));
        driverJoystick.leftTrigger().onTrue(new InstantCommand(intake::deployIntake));
        driverJoystick.leftBumper().onTrue(new InstantCommand(intake::undeployIntake));

        driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> brake));
        //driverJoystick.x().onTrue(new InstantCommand(shooter::stopMasterShooter));
        driverJoystick.a().whileTrue(new InstantCommand(intake::manuelUndeployIntake)).onFalse(new InstantCommand(intake::stopDeployIntake));
        driverJoystick.x().onTrue(new AgitateBalls(intake));

        //driverJoystick.y().whileTrue(new InstantCommand(spindexer::runSpindexer)).onFalse(new InstantCommand(spindexer::stopSpindexer));
        //driverJoystick.y().whileTrue(new InstantCommand(feeder::runFeeder)).onFalse(new InstantCommand(feeder::stopFeeder));

        driverJoystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverJoystick.povLeft().onTrue(new InstantCommand(shooter::decreaseShooterSpeed));
        driverJoystick.povRight().onTrue(new InstantCommand(shooter::increaseShooterSpeed));
        driverJoystick.povUp().onTrue(autoAaSM);
        driverJoystick.povDown().onTrue(autoAimFeed);

        //for tuning the drivetrain
        // driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureAutonOptions() {
        SendableRegistry.setName(autonChooser, "Autonomous Options");
        boolean redPath = true;
        boolean bluePath = false;

        autonChooser.addOption("RightToStation Red", new RightToStation(this, MaxSpeed, MaxAngularRate, redPath));
        autonChooser.addOption("RightToStation Blue", new RightToStation(this, MaxSpeed, MaxAngularRate, bluePath));

        autonChooser.addOption("LeftToDepot Red", new LeftToDepot(this, MaxSpeed, MaxAngularRate, redPath));
        autonChooser.addOption("LeftToDepot Blue", new LeftToDepot(this, MaxSpeed, MaxAngularRate, bluePath));

        autonChooser.addOption("ShootAuton Red", new ShootAuton(this, MaxSpeed, MaxAngularRate, redPath));
        autonChooser.addOption("ShootAuton Blue", new ShootAuton(this, MaxSpeed, MaxAngularRate, bluePath));

        autonChooser.addOption("LeftToMiddle Red", new LeftToMiddle(this, MaxSpeed, MaxAngularRate, redPath));
        autonChooser.addOption("LeftToMiddle Blue", new LeftToMiddle(this, MaxSpeed, MaxAngularRate, bluePath));

        autonChooser.addOption("FullMidSwipe Blue", new FullMidSwipe(this, 3, MaxAngularRate, bluePath));

        autonChooser.addOption("LeftMidShoot Blue", new LeftMidShoot(this, 3, MaxAngularRate, bluePath));

        autonChooser.addOption("LeftMidShootDepot Blue", new LeftMidShootDepot(this, 3, MaxAngularRate, bluePath));

        SmartDashboard.putData("Auto Choices", autonChooser);
    }

    public void autonomousInit() {
        initialize();
    }

    public Command getAutonomousCommand() {
        Command auton = autonChooser.getSelected();

        if (Localization.tagInView("limelight-a", "limelight-b")) {
            Localization.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b");
        } else {
            drivetrain.resetPose(((IAuto) auton).getInitialPose());
        }

        shooter.setDefaultCommand(new AutoAimAndShootMoving(this, hubLocation.getX(), hubLocation.getY()));
        return auton;
    }

    public void autonomousExit() {
    }

    public void robotPeriodic() {
        // LocalizationHelpers.updatePose(drivetrain, "limelight-a");
        // LocalizationHelpers.updatePose(drivetrain, "limelight-b");
        Localization.updatePose(drivetrain, "limelight-a", "limelight-b");

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
        shooter.removeDefaultCommand();
        turret.turretAimAtHubBool(false);
        feeder.stopFeeder();
        spindexer.stopSpindexer();
        intake.stopIntake();

        // If we are testing teleop - not connected to a real match then zero all mechanisms
        // if (DriverStation.isFMSAttached() == false) {
        //     // Intake - set position based on absolute encoder
        //     intake.resetDeployPosition();

        //     // Hood - set positon based on absolute encoder
        //     shooter.resetHoodPosition();

        //     // Turret - use command to move turret to zero position
        //     CommandScheduler.getInstance().schedule(new TurretResetHome(this));
        

        //     // Limelight - set robot position based on limelights
        //     LocalizationHelpers.resetToLimelightPose(drivetrain, "limelight-a", "limelight-b");
        // }
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
                autoAimFeed.setTarget(Constants.BLUE_FEED_ONE);
            } else {
                autoAimFeed.setTarget(Constants.BLUE_FEED_TWO);
            }
        } else {
            if (drivetrain.getPose().getY() > 4.0) {
                autoAimFeed.setTarget(Constants.RED_FEED_ONE);
            } else {
                autoAimFeed.setTarget(Constants.RED_FEED_TWO);
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
