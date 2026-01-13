package frc.robot;

import edu.wpi.first.networktables.*;

public class Constants {
    public static boolean hasInitializedFromVision = false;

    static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    static NetworkTable testTable = inst.getTable("LimeLight Testing");
    public static final BooleanPublisher visionValidFrontPub = testTable.getBooleanTopic("Front Vision Valid").publish();
    public static final DoublePublisher distanceToTagFrontPub = testTable.getDoubleTopic("Front Distance to Tag").publish();

    public static final DoublePublisher visionPoseXFrontPub = testTable.getDoubleTopic("Front Pose X").publish();
    public static final DoublePublisher visionPoseYFrontPub = testTable.getDoubleTopic("Front Pose Y").publish();
    public static final DoublePublisher visionPoseRotFrontPub = testTable.getDoubleTopic("Front Pose Rot").publish();

    public static final BooleanPublisher visionValidRearPub = testTable.getBooleanTopic("Rear Vision Valid").publish();
    public static final DoublePublisher distanceToTagRearPub = testTable.getDoubleTopic("Rear Distance to Tag").publish();

    public static final DoublePublisher visionPoseXRearPub = testTable.getDoubleTopic("Rear Pose X").publish();
    public static final DoublePublisher visionPoseYRearPub = testTable.getDoubleTopic("Rear Pose Y").publish();
    public static final DoublePublisher visionPoseRotRearPub = testTable.getDoubleTopic("Rear Pose Rot").publish();

    public static final DoublePublisher fusedPoseXPub = testTable.getDoubleTopic("Fused Pose X").publish();
    public static final DoublePublisher fusedPoseYPub = testTable.getDoubleTopic("Fused Pose Y").publish();
    public static final DoublePublisher fusedPoseRotPub = testTable.getDoubleTopic("Fused Pose Rot").publish();

    public static final DoublePublisher poseXError = testTable.getDoubleTopic("Pose X Error").publish();
    public static final DoublePublisher poseYError = testTable.getDoubleTopic("Pose Y Error").publish();
    public static final DoublePublisher poseRotError = testTable.getDoubleTopic("Pose Rot Error").publish();

    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_ANGLE_ID = 2;
    
    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_ANGLE_ID = 5;

    public static final int REAR_LEFT_DRIVE_ID = 7;
    public static final int REAR_LEFT_ANGLE_ID = 8;

    public static final int REAR_RIGHT_DRIVE_ID = 10;
    public static final int REAR_RIGHT_ANGLE_ID = 11;

    public static final int TEST_MOTOR_ID = 35;
    
    // Motor Limits
    public static final double MOTOR_STATOR_CURRENT_LIMIT = 60.0;
    public static final double MOTOR_TEMP_WARNING = 70.0;

}
