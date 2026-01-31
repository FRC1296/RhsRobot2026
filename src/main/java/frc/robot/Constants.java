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


    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_ANGLE_ID = 2;
    
    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_ANGLE_ID = 5;

    public static final int REAR_LEFT_DRIVE_ID = 7;
    public static final int REAR_LEFT_ANGLE_ID = 8;

    public static final int REAR_RIGHT_DRIVE_ID = 10;
    public static final int REAR_RIGHT_ANGLE_ID = 11;


     public static final int CAN_DEPLOY = 99;
    public static final int CAN_INTAKE = 98;
   
    public class turretConstants {
        public static boolean aimAtPose = false;
        public static final int TURRET_MOTOR_ID = 23;
    }

    
    // Motor Limits
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 60.0;
    public static final double MOTOR_TEMP_WARNING = 70.0;

}
