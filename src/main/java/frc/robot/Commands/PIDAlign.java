package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;

public class PIDAlign extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;

    private final PIDController xPID = new PIDController(10.0, 0, 0);
    private final PIDController yPID = new PIDController(10.0, 0, 0);
    private final PIDController thetaPID = new PIDController(8.0, 0, 0);

    private final SwerveRequest.FieldCentric pidRequest = new SwerveRequest.FieldCentric();


    public PIDAlign(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;

        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Pose2d current = drivetrain.getPose();

        double xSpeed = xPID.calculate(current.getX(), targetPose.getX());

        double ySpeed = yPID.calculate(current.getY(), targetPose.getY());

        double thetaSpeed = thetaPID.calculate(current.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        drivetrain.setControl(pidRequest.withVelocityX(xSpeed).withVelocityY(ySpeed)
                .withRotationalRate(thetaSpeed));
    }

    @Override
    public boolean isFinished() {
        Pose2d error = drivetrain.getPose().relativeTo(targetPose);

        return Math.abs(error.getX()) < 0.02 // 3 cm
                && Math.abs(error.getY()) < 0.02
                && Math.abs(error.getRotation().getDegrees()) < 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(
                () -> pidRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}
