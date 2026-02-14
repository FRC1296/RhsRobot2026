package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolationHelper {

    private static double shortRangeMax = 3.0;
    private static double mediumRangeMax = 5.5;

    private static double shortRangeHoodPosition = 0.0;
    private static double mediumRangeHoodPosition = 0.40;
    private static double longRangeHoodPosition = 0.75;

    private static InterpolatingDoubleTreeMap shortRangeTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap mediumRangeTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap longRangeTable = new InterpolatingDoubleTreeMap();

    static {
        shortRangeTable.put(1.0, 15.0); // 1m -> 15 RPS
        shortRangeTable.put(1.5, 18.0); // 1.5m -> 18 RPS
        shortRangeTable.put(2.0, 22.0); // 2m -> 22 RPS
        shortRangeTable.put(2.5, 26.0); // 2.5m -> 26 RPS
        shortRangeTable.put(3.0, 30.0); // 3m -> 30 RPS

        mediumRangeTable.put(3.0, 32.0);
        mediumRangeTable.put(3.5, 36.0);
        mediumRangeTable.put(4.0, 40.0);
        mediumRangeTable.put(4.5, 44.0);
        mediumRangeTable.put(5.0, 48.0);
        mediumRangeTable.put(5.5, 52.0);

        longRangeTable.put(5.5, 54.0);
        longRangeTable.put(6.0, 58.0);
        longRangeTable.put(6.5, 62.0);
        longRangeTable.put(7.0, 66.0);
        longRangeTable.put(7.5, 70.0);
        longRangeTable.put(8.0, 74.0);
    }

    public static double calculateShooterSpeed(double distance) {

        if (distance <= shortRangeMax) {
            // Short range: use short range table and hood position
            double velocity = shortRangeTable.get(distance);
            return velocity;
        } else if (distance <= mediumRangeMax) {
            // Medium range: use medium range table and hood position
            double velocity = mediumRangeTable.get(distance);
            return velocity;
        } else {
            // Long range: use long range table and hood position
            double velocity = longRangeTable.get(distance);
            return velocity;
        }
    }

    public static double calculateHoodPosition(double distance) {

        if (distance <= shortRangeMax) {
            return shortRangeHoodPosition;
        } else if (distance <= mediumRangeMax) {
            return mediumRangeHoodPosition;
        } else {
            return longRangeHoodPosition;
        }
    }
}
