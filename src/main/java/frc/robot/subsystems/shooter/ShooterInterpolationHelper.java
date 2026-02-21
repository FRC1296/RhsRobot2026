package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolationHelper {

    private static double shortRangeMax = 3.5;

    private static double shortRangeHoodPosition = 0.0;
    private static double mediumRangeHoodPosition = 0.25;

    private static InterpolatingDoubleTreeMap shortRangeTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap mediumRangeTable = new InterpolatingDoubleTreeMap();

    static {
        shortRangeTable.put(1.5, 48.0); // 1m -> 15 RPS
        shortRangeTable.put(2.0, 54.0); // 1.5m -> 18 RPS
        shortRangeTable.put(2.5, 60.0); // 2m -> 22 RPS
        shortRangeTable.put(3.0, 72.0); // 2.5m -> 26 RPS
        shortRangeTable.put(3.5, 80.0); // 3m -> 30 RPS

        mediumRangeTable.put(3.5, 70.0);
        mediumRangeTable.put(4.0, 76.0);
        mediumRangeTable.put(4.5, 82.0);
        mediumRangeTable.put(5.0, 84.0);
        mediumRangeTable.put(5.7, 91.0);
    }

    public static double calculateShooterSpeed(double distance) {

        if (distance <= shortRangeMax) {
            // Short range: use short range table and hood position
            double velocity = shortRangeTable.get(distance);
            return velocity;
        } else {
            // Medium range: use medium range table and hood position
            double velocity = mediumRangeTable.get(distance);
            return velocity;
        }
    }

    public static double calculateHoodPosition(double distance) {

        if (distance <= shortRangeMax) {
            return shortRangeHoodPosition;
        } else {
            return mediumRangeHoodPosition;
        }
    }
}
