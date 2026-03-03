package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterInterpolationHelper extends SubsystemBase {

    protected InterpolatingDoubleTreeMap shooterSpeedShortTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap shooterSpeedLongTable = new InterpolatingDoubleTreeMap();

    //protected InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
    protected InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();

    public ShooterInterpolationHelper(String name) {
        super(name);
        initializeTables();
    }

    public void initializeTables() {
        shooterSpeedShortTable.put(1.5, 31.0);
        shooterSpeedShortTable.put(1.75, 32.0); 
        shooterSpeedShortTable.put(2.0, 33.0);
        shooterSpeedShortTable.put(2.5, 35.0); 

        shooterSpeedLongTable.put(3.0, 37.0);
        shooterSpeedLongTable.put(3.5, 41.0);
        shooterSpeedLongTable.put(4.0, 45.0);
        shooterSpeedLongTable.put(4.5, 30.0);

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
        double speed = 40.0;
        if (distance <= 3.0) {
            speed = shooterSpeedShortTable.get(distance);
        } else {
            speed = shooterSpeedLongTable.get(distance);
        }
        return speed;       
    }

    public double calculateHoodPosition(double distance) {
        double hood = 0.09;
        if (distance <= 3) {
            hood = 0.09;
        } else {
            hood = 0.3;
        }
        return hood;

    }
}
