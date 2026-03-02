package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolationHelper {

    private InterpolatingDoubleTreeMap shooterSpeedTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();

    public ShooterInterpolationHelper() {
        initializeTables();
    }

    public void initializeTables() {
        shooterSpeedTable.put(1.5, 30.0); // 1m -> 15 RPS
        shooterSpeedTable.put(2.0, 35.0);
        shooterSpeedTable.put(2.5, 67.0);
        shooterSpeedTable.put(3.0, 72.0);
        shooterSpeedTable.put(3.5, 80.0);

        hoodTable.put(1.5, 0.05); //Distance to hub -> Hood Position
        hoodTable.put(2.0, 0.10);
        hoodTable.put(2.5, 0.15);
        hoodTable.put(3.0, 0.20);
        hoodTable.put(3.5, 0.25);

        timeOfFlightTable.put(1.5, 5.0); //Distance to hub -> Time of Flight sec
        timeOfFlightTable.put(2.0, 5.5);
        timeOfFlightTable.put(2.5, 6.0);
        timeOfFlightTable.put(3.0, 6.5);
        timeOfFlightTable.put(3.5, 7.5);
    }

    public double calculateToF(double distance) {
        return timeOfFlightTable.get(distance);
    }

    public double calculateShooterSpeed(double distance) {
        return shooterSpeedTable.get(distance);
        // if (distance <= shortRangeMax) {
        //     double velocity = shortRangeTable.get(distance);
        //     return velocity;
        // } else {
        //     double velocity = mediumRangeTable.get(distance);
        //     return velocity;
        // }
    }

    public double calculateHoodPosition(double distance) {
        return hoodTable.get(distance);
        // double hoodpos = 0.055;
        
        // if(distance <= 1.5){
        //     return hoodpos;
        // } else if(distance > 6.0){
        //     return 0.5;
        // } else {
        //     return ((0.043333 * distance) - 0.01);
        // }

    }
}
