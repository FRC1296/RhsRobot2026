package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterInterpolationHelper extends SubsystemBase {

    protected InterpolatingDoubleTreeMap shooterSpeedTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
    protected double shooterInterpSpeedAdjustment = 0;
    protected double ToFInterpAdjustment = 0;

    public ShooterInterpolationHelper(String name) {
        super(name);
        initializeTables();
    }

    public void initializeTables() {
        shooterSpeedTable.put(1.5, 29.5);
        shooterSpeedTable.put(2.0, 32.0);
        shooterSpeedTable.put(2.5, 35.0);
        shooterSpeedTable.put(3.0, 38.5);
        shooterSpeedTable.put(3.5, 41.0);
        shooterSpeedTable.put(4.0, 45.0);
        shooterSpeedTable.put(4.5, 52.5);
        shooterSpeedTable.put(5.265, 56.0);

        hoodTable.put(1.5, 0.0);
        hoodTable.put(6.0, 0.35);

        //Distance to hub -> Time of Flight sec
        timeOfFlightTable.put(1.5, 0.77);
        timeOfFlightTable.put(2.0, 0.84);
        timeOfFlightTable.put(2.5, 0.96);
        timeOfFlightTable.put(3.0, 1.11);
        timeOfFlightTable.put(3.5, 1.19);
        timeOfFlightTable.put(4.0, 1.30);
        timeOfFlightTable.put(4.5, 1.68);
        timeOfFlightTable.put(5.265, 1.76);
    }

    public double calculateToF(double distance) {
        return timeOfFlightTable.get(distance) - ToFInterpAdjustment;
    }

    public double calculateShooterSpeed(double distance) {
        return shooterSpeedTable.get(distance) + shooterInterpSpeedAdjustment;
    }

    public double calculateHoodPosition(double distance) {
        double hood = 0.0;
        if (distance <= 1.5) {
            hood = 0;
        } else if (distance >= 6.0){
            hood = 0.35;
        } else {
            hood = hoodTable.get(distance);
        }
        return hood;
    }
}
