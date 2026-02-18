package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface IAuto {
    // The PID for x component of our robot
    public PIDController xController = new PIDController(1.0, 0, 0);
    // The PID for y component of our robot
    public PIDController yController = new PIDController(1.0, 0, 0);
    // The PID for direction of our robot
    public TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    // public ProfiledPIDController thetaController = new ProfiledPIDController(1.0, 0, 0,
    // thetaControllerConstraints);
    public PIDController thetaController = new PIDController(1.0, 0, 0);

    /**
     * This method will return the initial pose for our robot. It will most likely come from the
     * trajectory information.
     * 
     * @return Pose2d representation of initial position of our robot
     */
    public Pose2d getInitialPose();
}
