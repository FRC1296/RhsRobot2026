package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.lang.ProcessBuilder.Redirect;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autonomous.DefaultAuton;
import frc.robot.autonomous.FullMidSwipe;
import frc.robot.autonomous.IAuto;
import frc.robot.autonomous.LeftMidShoot;
import frc.robot.autonomous.LeftMidShootDepot;
import frc.robot.autonomous.LeftToDepot;
import frc.robot.autonomous.LeftToMiddle;
import frc.robot.autonomous.RightMidShoot;
import frc.robot.autonomous.RightToStation;
import frc.robot.autonomous.ShootAuton;
import frc.robot.autonomous.Test1;
import frc.robot.commands.AutoAimAndShootMoving;
import frc.robot.commands.FeedAimAndShootMoving;
import frc.robot.commands.RobotAimAtHub;
import frc.robot.commands.ShootBalls;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hid.ControlPanel;
import frc.robot.subsystems.intake.AgitateBalls;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;


public class FMJRobotContainer {

    private double MaxSpeed = Constants.driveConstants.driveSpeed;
    private double MaxAngularRate = Constants.driveConstants.rotationSpeed;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);
    private final CommandXboxController testJoystick = new CommandXboxController(3);
    private final ControlPanel controlPanel = new ControlPanel(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private FeederSubsystem feeder;
    private ClimberSubsystem climber;
    private SpindexerSubsystem spindexer;
    private Vision vision;
    //private LedSubsystem LED;

    private AutoAimAndShootMoving autoAaSM;
    private FeedAimAndShootMoving autoAimFeed;
    private RobotAimAtHub autoRobotHub;
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
    private DoublePublisher robotDistanceToFeedPublisher;
    private BooleanPublisher activePhasePublisher;
    private DoublePublisher matchTimePublisher;
    private DoublePublisher timeLeftInShift;

    private SendableChooser<Command> autonChooser = new SendableChooser<>();

    private boolean initialized = false;

    public FMJRobotContainer() {
        shooter = new ShooterSubsystem(drivetrain);
        turret = new TurretSubsystem(drivetrain);
        intake = new IntakeSubsystem();
        feeder = new FeederSubsystem(drivetrain);
        climber = new ClimberSubsystem();
        spindexer = new SpindexerSubsystem(this);
        //LED = new LedSubsystem();

        switch (Constants.currentMode) {
            case REAL:
                vision = new Vision(
                        drivetrain::addVisionMeasurement,
                        new VisionIOLimelight("limelight-a", drivetrain::getRotation),
                        new VisionIOLimelight("limelight-b", drivetrain::getRotation));
                break;
            case SIM:
                vision = new Vision(
                        drivetrain::addVisionMeasurement,
                        new VisionIO() {},
                        new VisionIO() {});
                break;
            case REPLAY:
            default:
                vision = new Vision(
                        drivetrain::addVisionMeasurement,
                        new VisionIO() {},
                        new VisionIO() {});
                break;
        }

        shootBalls = new ShootBalls(this);

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            hubLocation = Constants.BLUE_HUB;
            feedLocation = Constants.BLUE_FEED_ONE;
        } else {
            hubLocation = Constants.RED_HUB;
            feedLocation = Constants.RED_FEED_ONE;
        }

        autoRobotHub = new RobotAimAtHub(this);
        autoAimFeed = new FeedAimAndShootMoving(this);
        autoAaSM = new AutoAimAndShootMoving(this);

        NamedCommands.registerCommand("runIntake", new InstantCommand(intake::runIntake));
        NamedCommands.registerCommand("runFeeder", new InstantCommand(feeder::runFeeder, feeder));
        NamedCommands.registerCommand("runSpindexer", new InstantCommand(spindexer::runSpindexer));
        NamedCommands.registerCommand("stopSpindexer", new InstantCommand(spindexer::stopSpindexer));
        NamedCommands.registerCommand("stopIntake", new InstantCommand(intake::stopIntake));
        NamedCommands.registerCommand("stopFeeder", new InstantCommand(feeder::stopFeeder));
        NamedCommands.registerCommand("deployIntake", new InstantCommand(intake::deployIntake));
        NamedCommands.registerCommand("undeployIntake", new InstantCommand(intake::undeployIntake));
        NamedCommands.registerCommand("agitate", new AgitateBalls(intake));

        configureNetworkTable();
        configureDriverBindings();
        configureOperatorBindings();
        //configureControlPanelBindings();
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
        robotDistanceToFeedPublisher = robotTable.getDoubleTopic("Robot Distance to Feed").publish();
        
        NetworkTable FMSTable = robotTable.getSubTable("FMS");
        activePhasePublisher = FMSTable.getBooleanTopic("Active").publish();
        matchTimePublisher = FMSTable.getDoubleTopic("Match Time").publish();
        timeLeftInShift = FMSTable.getDoubleTopic("Time Left In Shift").publish();
    }

    private void configureOperatorBindings() {

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
        .onFalse(new InstantCommand(feeder::reverseFeeder).withTimeout(1.0).andThen(new InstantCommand(feeder::stopFeeder)));

        operatorJoystick.x().onTrue(new InstantCommand(() -> turret.turretAimAtHubBool(false)));
        operatorJoystick.b().onTrue(new InstantCommand(() -> turret.turretAimToFeedBool(false)));

        operatorJoystick.povRight().onTrue(new InstantCommand(turret::increaseTurretAngle));
        operatorJoystick.povLeft().onTrue(new InstantCommand(turret::decreaseTurretAngle));
        operatorJoystick.povUp().onTrue(autoAaSM);
        operatorJoystick.povDown().onTrue(autoAimFeed);

        operatorJoystick.back().onTrue(new InstantCommand(intake::resetDeployPosition));
    }

    private void configureControlPanelBindings() {

        controlPanel.shootBalls().toggleOnTrue(shootBalls);
        controlPanel.reverseSpindexer().whileTrue(new ParallelCommandGroup(
            new InstantCommand(spindexer::reverseSpindexer),
            new InstantCommand(feeder::reverseFeeder)
        )).onFalse(new ParallelCommandGroup(
            new InstantCommand(spindexer::stopSpindexer),
            new InstantCommand(feeder::stopFeeder)
        ));
        controlPanel.manuelDeployIntake()
            .whileTrue(new InstantCommand(intake::manuelDeployIntake))
            .onFalse(new InstantCommand(intake::stopDeployIntake));
        controlPanel.runSpindexerFeeder()
            .whileTrue(new InstantCommand(spindexer::runSpindexer))
            .onFalse(new InstantCommand(spindexer::stopSpindexer));
        controlPanel.runSpindexerFeeder()
            .whileTrue(new InstantCommand(feeder::runFeeder))
            .onFalse(new InstantCommand(feeder::reverseFeeder).withTimeout(1.0).andThen(new InstantCommand(feeder::stopFeeder)));

        controlPanel.turretAimHub().onTrue(new InstantCommand(() -> turret.turretAimAtHubBool(false)));
        controlPanel.turretAimFeed().onTrue(new InstantCommand(() -> turret.turretAimToFeedBool(false)));

        controlPanel.turretAngleRight().onTrue(new InstantCommand(turret::increaseTurretAngle));
        controlPanel.turretAngleLeft().onTrue(new InstantCommand(turret::decreaseTurretAngle));
        controlPanel.autoAimShoot().onTrue(autoAaSM);
        controlPanel.autoAimFeed().onTrue(autoAimFeed);
    }

    private void configureDriverBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverJoystick.getRightX() * Math.toRadians(MaxAngularRate))));
        driverJoystick.y().whileTrue(drivetrain.applyRequest(() -> drive
            .withVelocityX(-driverJoystick.getLeftY() * MaxSpeed * 0.5)
            .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed * 0.5)
            .withRotationalRate(-driverJoystick.getRightX() * Math.toRadians(MaxAngularRate) * 1.5  )));
        driverJoystick.y().whileTrue(new ParallelCommandGroup(
            new InstantCommand(spindexer::runSpindexer),
            new InstantCommand(feeder::runFeeder)
        )).onFalse(new ParallelCommandGroup(
            new InstantCommand(spindexer::stopSpindexer),
            new InstantCommand(feeder::stopFeeder)
        ));

        driverJoystick.rightTrigger().whileTrue(new InstantCommand(intake::runIntake)).onFalse(new InstantCommand(intake::stopIntake));
        driverJoystick.rightBumper().whileTrue(new InstantCommand(intake::runIntakeReverse)).onFalse(new InstantCommand(intake::stopIntake));
        driverJoystick.leftTrigger().onTrue(new InstantCommand(intake::deployIntake));
        driverJoystick.leftBumper().onTrue(new InstantCommand(intake::undeployIntake));

        driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoystick.a().whileTrue(new InstantCommand(intake::manuelUndeployIntake)).onFalse(new InstantCommand(intake::stopDeployIntake));
        driverJoystick.x().onTrue(new AgitateBalls(intake));

        driverJoystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverJoystick.povLeft().onTrue(new InstantCommand(shooter::decreaseShooterSpeed));
        driverJoystick.povRight().onTrue(new InstantCommand(shooter::increaseShooterSpeed));
        driverJoystick.povUp().onTrue(new InstantCommand(shooter::increaseShooterInterpSpeed));
        driverJoystick.povDown().onTrue(new InstantCommand(shooter::decreaseShooterInterpSpeed));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureTempBindings() {
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    public void configureAutonOptions() {
        SendableRegistry.setName(autonChooser, "Autonomous Options");
        boolean redPath = true;
        boolean bluePath = false;

        // autonChooser.addOption("RightToStation Red", new RightToStation(this, MaxSpeed, MaxAngularRate, redPath));
        // autonChooser.addOption("RightToStation Blue", new RightToStation(this, MaxSpeed, MaxAngularRate, bluePath));

        autonChooser.addOption("LeftToDepot Red", new LeftToDepot(this, MaxSpeed, MaxAngularRate, redPath));
        autonChooser.addOption("LeftToDepot Blue", new LeftToDepot(this, MaxSpeed, MaxAngularRate, bluePath));

        autonChooser.addOption("ShootAuton Red", new ShootAuton(this, MaxSpeed, MaxAngularRate, redPath));
        autonChooser.addOption("ShootAuton Blue", new ShootAuton(this, MaxSpeed, MaxAngularRate, bluePath));

        // autonChooser.addOption("LeftToMiddle Red", new LeftToMiddle(this, MaxSpeed, MaxAngularRate, redPath));
        // autonChooser.addOption("LeftToMiddle Blue", new LeftToMiddle(this, MaxSpeed, MaxAngularRate, bluePath));

        // autonChooser.addOption("FullMidSwipe Blue", new FullMidSwipe(this, 3, MaxAngularRate, bluePath));

        autonChooser.addOption("LeftMidShoot Blue", new LeftMidShoot(this, 3, MaxAngularRate, bluePath));
        autonChooser.addOption("LeftMidShoot Red", new LeftMidShoot(this, 3, MaxAngularRate, redPath));

        autonChooser.addOption("LeftMidShootDepot Blue", new LeftMidShootDepot(this, 3, MaxAngularRate, bluePath));
        autonChooser.addOption("LeftMidShootDepot Red", new LeftMidShootDepot(this, 3, MaxAngularRate, redPath));
   
        autonChooser.addOption("RightMidShoot Blue", new RightMidShoot(this, 3, MaxAngularRate, bluePath));
        autonChooser.addOption("RightMidShoot Red", new RightMidShoot(this, 3, MaxAngularRate, redPath));

        // autonChooser.addOption("MidTest Blue", new Test1(this, 3, MaxAngularRate, bluePath));
        // autonChooser.addOption("MidTest Red", new Test1(this, 3, MaxAngularRate, redPath));

        SmartDashboard.putData("Auto Choices", autonChooser);
    }

    public void autonomousInit() {
        initialize();
    }

    public Command getAutonomousCommand() {
        Command auton = autonChooser.getSelected();

        if(auton == null) {
            if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                auton = new DefaultAuton(this, MaxSpeed, MaxAngularRate, false);
            } else {
                auton = new DefaultAuton(this, MaxSpeed, MaxAngularRate, true);
            }
        } 

        drivetrain.resetPose(((IAuto) auton).getInitialPose());

        shooter.setDefaultCommand(new AutoAimAndShootMoving(this, hubLocation.getX(), hubLocation.getY()));
        return auton;
    }

    public void autonomousExit() {
    }

    public void robotPeriodic() {
        robotVelocityPublisher.set(robotVelocity);
        robotAnglePublisher.set(robotAngle);

        shooterVelocityPublisher.set(shooter.getShooterVelocity());
        turretAnglePublisher.set(turret.getTurretAngle());

        if (hubLocation != null) {
            Translation2d robotLocation = drivetrain.getPose().plus(shooter.getShooterOffset()).getTranslation();
            robotDistanceToHubPublisher.set(robotLocation.getDistance(hubLocation));
        }

        if (feedLocation != null) {
            Translation2d robotLocation = drivetrain.getPose().plus(shooter.getShooterOffset()).getTranslation();
            robotDistanceToFeedPublisher.set(robotLocation.getDistance(feedLocation));
        }

        try {
            activePhasePublisher.set(ShiftHelpers.getOfficialShiftInfo().active());
            matchTimePublisher.set(DriverStation.getMatchTime());
            timeLeftInShift.set(ShiftHelpers.getOfficialShiftInfo().remainingTime());
        } catch (Throwable t) {
            System.out.println("Can't Publish Shifted Shift");
            throw t;
        }
    }

    public void teleopInit() {
        initialize();
        shooter.removeDefaultCommand();
        turret.turretAimAtHubBool(false);
        feeder.stopFeeder();
        spindexer.stopSpindexer();
        intake.stopIntake();
    }

    public void teleopPeriodic() {
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