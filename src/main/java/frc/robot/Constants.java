package frc.robot;

import edu.wpi.first.networktables.*;

public class Constants {
    public static boolean hasInitializedFromVision = false;

    static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    static NetworkTable testTable = inst.getTable("LimeLight Testing");
    public static final BooleanPublisher visionValidAPub = testTable.getBooleanTopic("A Vision Valid").publish();
    public static final DoublePublisher distanceToTagAPub = testTable.getDoubleTopic("A Distance to Tag").publish();

    public static final DoublePublisher visionPoseXAPub = testTable.getDoubleTopic("A Pose X").publish();
    public static final DoublePublisher visionPoseYAPub = testTable.getDoubleTopic("A Pose Y").publish();
    public static final DoublePublisher visionPoseRotAPub = testTable.getDoubleTopic("A Pose Rot").publish();

    public static final BooleanPublisher visionValidBPub = testTable.getBooleanTopic("B Vision Valid").publish();
    public static final DoublePublisher distanceToTagBPub = testTable.getDoubleTopic("B Distance to Tag").publish();

    public static final DoublePublisher visionPoseXBPub = testTable.getDoubleTopic("B Pose X").publish();
    public static final DoublePublisher visionPoseYBPub = testTable.getDoubleTopic("B Pose Y").publish();
    public static final DoublePublisher visionPoseRotBPub = testTable.getDoubleTopic("B Pose Rot").publish();

    public static final DoublePublisher fusedPoseXPub = testTable.getDoubleTopic("Fused Pose X").publish();
    public static final DoublePublisher fusedPoseYPub = testTable.getDoubleTopic("Fused Pose Y").publish();
    public static final DoublePublisher fusedPoseRotPub = testTable.getDoubleTopic("Fused Pose Rot").publish();

    

    public class driveConstants{

        public static final int FRONT_RIGHT_DRIVE_ID = 1;
        public static final int FRONT_RIGHT_ANGLE_ID = 2;
        public static final int FRONT_RIGHT_CANCODER = 3;

        public static final int FRONT_LEFT_DRIVE_ID = 4;
        public static final int FRONT_LEFT_ANGLE_ID = 5;
        public static final int FRONT_LEFT_CANCODER = 6;

        public static final int REAR_RIGHT_DRIVE_ID = 7;
        public static final int REAR_RIGHT_ANGLE_ID = 8;
        public static final int REAR_RIGHT_CANCODER = 9;

        public static final int REAR_LEFT_DRIVE_ID = 10;
        public static final int REAR_LEFT_ANGLE_ID = 11;
        public static final int REAR_LEFT_CANCODER = 12;

        public static boolean driveAimAtHub = false;

        public static final double driveSpeed = 0.5; // meters per sec
        public static final double rotationSpeed = 1.0; // rotations per sec
    }
    
    public class turretConstants {
        public static boolean turretAimAtHub = false;
        public static boolean turretAimToFeed = false;
        public static final int TURRET_MOTOR_ID = 23;
    }

    public class shooterConstants {
        public static final int HOOD_MOTOR_ID = 20;
        public static final int SHOOTER_MASTER_MOTOR_ID = 21;
        public static final int SHOOTER_SLAVE_MOTOR_ID = 22;
        public static final int HOOD_ENCODER_ID = 24;
        public static final int BALL_COUNT_ID = 25;

        public static boolean shooterInterpolate = false;
    }

    public class intakeConstants {
        public static final int INTAKE_DEPLOY_MOTOR_ID = 13;
        public static final int INTAKE_ROLLER_MOTOR_ID = 14;
        public static final int INTAKE_ENCODER_ID = 26;
    }

    public class feederConstants {
        public static final int SPINDEXER_MOTOR_ID = 15;
        public static final int FEEDER_MOTOR_ID = 16;
    }

}
