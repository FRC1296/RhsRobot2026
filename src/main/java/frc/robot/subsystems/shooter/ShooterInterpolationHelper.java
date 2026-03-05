package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterInterpolationHelper extends SubsystemBase {

    protected InterpolatingDoubleTreeMap shooterSpeedTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
    protected double shooterInterpSpeedAdjustment = 0;


    public ShooterInterpolationHelper(String name) {
        super(name);
        initializeTables();
    }

    public void initializeTables() {
        shooterSpeedTable.put(1.5, 34.0);
        shooterSpeedTable.put(2.0, 37.0);
        shooterSpeedTable.put(2.5, 40.0);
        shooterSpeedTable.put(3.0, 43.0);
        shooterSpeedTable.put(3.5, 46.0);
        shooterSpeedTable.put(4.0, 48.0);
        shooterSpeedTable.put(4.5, 51.0);
        shooterSpeedTable.put(5.68, 54.0);

        hoodTable.put(1.5, 0.0);
        hoodTable.put(6.0, 0.35);

        timeOfFlightTable.put(1.5, 5.0); //Distance to hub -> Time of Flight sec
    }

    public double calculateToF(double distance) {
        return timeOfFlightTable.get(distance);
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
