package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterInterpolationHelper extends SubsystemBase {

    protected InterpolatingDoubleTreeMap shooterSpeedTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
    protected double shooterInterpSpeedAdjustment = 0.0;
    protected double ToFInterpAdjustment = 0;

    public ShooterInterpolationHelper(String name) {
        super(name);
        initializeTables();
    }

    public void initializeTables() {
        shooterSpeedTable.put(1.25, 30.0);
        shooterSpeedTable.put(1.5, 33.25);
        shooterSpeedTable.put(2.0, 34.0);
        shooterSpeedTable.put(2.5, 35.5);
        shooterSpeedTable.put(3.0, 37.0);
        shooterSpeedTable.put(3.5, 38.75);
        shooterSpeedTable.put(4.0, 40.5);
        shooterSpeedTable.put(4.5, 43.0);
        shooterSpeedTable.put(5.0, 44.5);
        shooterSpeedTable.put(5.40, 46.5);
        shooterSpeedTable.put(6.0, 47.5);
        shooterSpeedTable.put(6.5, 50.0);

        hoodTable.put(1.5, 0.05);
        hoodTable.put(5.5, 0.625);

        //Distance to hub -> Time of Flight sec        
        timeOfFlightTable.put(1.25, 0.98);
        timeOfFlightTable.put(1.5, 1.15);
        timeOfFlightTable.put(2.0, 1.16);
        timeOfFlightTable.put(2.5, 1.18);
        timeOfFlightTable.put(3.0, 1.20);
        timeOfFlightTable.put(3.5, 1.06);
        timeOfFlightTable.put(4.0, 1.10);
        timeOfFlightTable.put(4.5, 1.18);
        timeOfFlightTable.put(5.0, 1.15);
        timeOfFlightTable.put(5.45, 1.12);
        timeOfFlightTable.put(6.0, 1.26);
        timeOfFlightTable.put(6.5, 1.27);


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
            hood = 0.625;
        } else {
            hood = hoodTable.get(distance);
        }
        return hood;
    }
}
