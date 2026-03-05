package frc.robot.subsystems.hid;

import edu.wpi.first.wpilibj.GenericHID;


/** Add your docs here. */
public class ControlPanel extends GenericHID {

    private final int OPERATOR_MODE = 1;

    public ControlPanel(int port) {
        super(port);
    }

    public boolean getOperatorMode() {
        return this.getRawButton(OPERATOR_MODE);
    }

}