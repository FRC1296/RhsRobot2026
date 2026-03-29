package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterInterpolationHelper extends SubsystemBase {

    protected InterpolatingDoubleTreeMap shooterSpeedTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
    protected double shooterInterpSpeedAdjustment = 0.5;
    protected double ToFInterpAdjustment = 0;

    public ShooterInterpolationHelper(String name) {
        super(name);
        initializeTables();
    }

    public void initializeTables() {
        shooterSpeedTable.put(1.25, 28.0);
        shooterSpeedTable.put(1.5, 29.0);
        shooterSpeedTable.put(2.0, 33.0);
        shooterSpeedTable.put(2.5, 35.0);
        shooterSpeedTable.put(3.0, 37.0);
        shooterSpeedTable.put(3.5, 38.5);
        shooterSpeedTable.put(4.0, 42.0);
        shooterSpeedTable.put(4.5, 43.5);
        shooterSpeedTable.put(5.0, 46.5);
        shooterSpeedTable.put(5.45, 47.0);

        hoodTable.put(1.5, 0.05);
        hoodTable.put(5.5, 0.45);

        //Distance to hub -> Time of Flight sec        
        timeOfFlightTable.put(1.25, 0.77);
        timeOfFlightTable.put(1.5, 0.77);
        timeOfFlightTable.put(2.0, 1.00);
        timeOfFlightTable.put(2.5, 1.10);
        timeOfFlightTable.put(3.0, 1.00);
        timeOfFlightTable.put(3.5, 1.13);
        timeOfFlightTable.put(4.0, 1.27);
        timeOfFlightTable.put(4.5, 1.33);
        timeOfFlightTable.put(5.0, 1.4);
        timeOfFlightTable.put(5.45, 1.45);


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
        } else if (distance >= 5.5){
            hood = 0.4;
        } else {
            hood = hoodTable.get(distance);
        }
        return hood;
    }
}
