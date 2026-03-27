package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public static boolean hasInitializedFromVision = false;
    //location constants
    public static final Translation2d BLUE_HUB = new Translation2d(4.6, 4.0);
    public static final Translation2d RED_HUB = new Translation2d(11.9, 4.0);
    public static final Translation2d BLUE_FEED_ONE = new Translation2d(2.0, 6.5);
    public static final Translation2d BLUE_FEED_TWO = new Translation2d(2.0, 1.5);
    public static final Translation2d RED_FEED_ONE = new Translation2d(14.5, 6.5);
    public static final Translation2d RED_FEED_TWO = new Translation2d(14.5, 1.5);

    // Network Table Strings
    public static final String NETWORK_TABLE = "Robot Data";
    public static final String NT_ROBOT_VELOCITY = "Robot Velocity";
    public static final String NT_ROBOT_ANGLE = "Robot Angle";
    public static final String NT_ROBOT_DISTANCE_TO_HUB = "Robot Distance to Hub";

    public static final String NT_DRIVE = "Drive Subsystem";
    public static final String NT_DRIVE_VELOCITY = "Robot Velocity";

    public static final String NT_INTAKE = "Intake Subsystem";
    public static final String NT_INTAKE_POSITION = "Intake Position";
    public static final String NT_INTAKE_HAS_BALL = "Have ball";

    public static final String NT_SPINDEXER = "Spindexer Subsystem";
    public static final String NT_SPINDEXER_STALL = "Spindexer Stall";

    public static final String NT_SHOOTER = "Shooter Subsystem";
    public static final String NT_SHOOTER_HOOD_POSITION = "Hood Position";
    public static final String NT_SHOOTER_VELOCITY = "Shooter Velocity";
    public static final String NT_SHOOTER_DISTANCE_TO_HUB = "Distance To Hub";

    public static final String NT_TURRET_ANGLE = "Turret Angle";
    
    public class driveConstants{

        public static boolean driveAimAtHub = false;

        public static final double driveSpeed = 2.0; // meters per sec
        public static final double rotationSpeed = 2.25; // rotations per sec
    }
    
    public class turretConstants {
        public static boolean turretAimAtHub = false;
        public static boolean turretAimToFeed = false;
        public static final int TURRET_MOTOR_ID = 23;
        public static final int HALL_EFFECT_ID = 0;
    }

    public class shooterConstants {
        public static final int HOOD_MOTOR_ID = 20;
        public static final int SHOOTER_MASTER_MOTOR_ID = 21;
        public static final int SHOOTER_SLAVE_MOTOR_ID = 22;
        public static final int HOOD_ENCODER_ID = 24;
        public static final int BALL_COUNT_ID = 25;

        public static boolean shooterInterpolate = false;
        public static boolean shootWhileMoving = false;
    }

    public class intakeConstants {
        public static final int INTAKE_DEPLOY_MOTOR_ID = 13;
        public static final int INTAKE_ROLLER_MOTOR_ID = 14;
        public static final int INTAKE_ENCODER_ID = 26;
    }

    public class feederConstants {
        public static final int SPINDEXER_MOTOR_ID = 15;
        public static final int FEEDER_MOTOR_ID = 16;
        
        public static final double feederSpeed = 90.0;
        public static final double SPINDEXER_SPEED = 72.0;

    }

    public class climberConstants {
        public static final int CLIMBER_MOTOR_ID = 27;
    }
}
