package frc.robot.subsystems.hid;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** HID control panel with analog buttons wired into raw button inputs.
 *  Button numbers match the physical wire positions on the board.
 *  Adjust the constants below to match your actual wiring. */
public class ControlPanel extends GenericHID {

    // ── Wiring map ──────────────────────────────────────────────
    // Change these numbers to match which wire goes to which input
    // on your HID board.
    private static final int OPERATOR_MODE       = 1;

    // Operator-equivalent buttons
    private static final int SHOOT_BALLS         = 2;  // operatorJoystick.leftTrigger()   → toggleOnTrue(shootBalls)
    private static final int REVERSE_SPINDEXER   = 3;  // operatorJoystick.leftBumper()    → reverse spindexer + feeder
    private static final int MANUEL_DEPLOY_INTAKE = 4; // operatorJoystick.rightTrigger()  → manuelDeployIntake
    private static final int RUN_SPINDEXER_FEEDER = 5; // operatorJoystick.rightBumper()   → run spindexer + feeder
    private static final int TURRET_AIM_HUB      = 6;  // operatorJoystick.x()             → turretAimAtHubBool(false)
    private static final int TURRET_AIM_FEED     = 7;  // operatorJoystick.b()             → turretAimToFeedBool(false)
    private static final int TURRET_ANGLE_RIGHT  = 8;  // operatorJoystick.povRight()      → increaseTurretAngle
    private static final int TURRET_ANGLE_LEFT   = 9;  // operatorJoystick.povLeft()       → decreaseTurretAngle
    private static final int AUTO_AIM_SHOOT      = 10; // operatorJoystick.povUp()         → autoAaSM
    private static final int AUTO_AIM_FEED       = 11; // operatorJoystick.povDown()       → autoAimFeed
    private static final int RESET_DEPLOY        = 12; // operatorJoystick.back()          → resetDeployPosition
    // ─────────────────────────────────────────────────────────────

    public ControlPanel(int port) {
        super(port);
    }

    // ── Mode ─────────────────────────────────────────────────────
    public boolean getOperatorMode() {
        return getRawButton(OPERATOR_MODE);
    }

    public Trigger operatorMode() {
        return new Trigger(this::getOperatorMode);
    }

    // ── Operator-equivalent triggers ─────────────────────────────

    /** Equivalent to operatorJoystick.leftTrigger() — toggles shootBalls */
    public Trigger shootBalls() {
        return new Trigger(() -> getRawButton(SHOOT_BALLS));
    }

    /** Equivalent to operatorJoystick.leftBumper() — reverses spindexer + feeder */
    public Trigger reverseSpindexer() {
        return new Trigger(() -> getRawButton(REVERSE_SPINDEXER));
    }

    /** Equivalent to operatorJoystick.rightTrigger() — manuel deploy intake */
    public Trigger manuelDeployIntake() {
        return new Trigger(() -> getRawButton(MANUEL_DEPLOY_INTAKE));
    }

    /** Equivalent to operatorJoystick.rightBumper() — runs spindexer + feeder */
    public Trigger runSpindexerFeeder() {
        return new Trigger(() -> getRawButton(RUN_SPINDEXER_FEEDER));
    }

    /** Equivalent to operatorJoystick.x() — turret aim at hub */
    public Trigger turretAimHub() {
        return new Trigger(() -> getRawButton(TURRET_AIM_HUB));
    }

    /** Equivalent to operatorJoystick.b() — turret aim to feed */
    public Trigger turretAimFeed() {
        return new Trigger(() -> getRawButton(TURRET_AIM_FEED));
    }

    /** Equivalent to operatorJoystick.povRight() — increase turret angle */
    public Trigger turretAngleRight() {
        return new Trigger(() -> getRawButton(TURRET_ANGLE_RIGHT));
    }

    /** Equivalent to operatorJoystick.povLeft() — decrease turret angle */
    public Trigger turretAngleLeft() {
        return new Trigger(() -> getRawButton(TURRET_ANGLE_LEFT));
    }

    /** Equivalent to operatorJoystick.povUp() — auto aim and shoot */
    public Trigger autoAimShoot() {
        return new Trigger(() -> getRawButton(AUTO_AIM_SHOOT));
    }

    /** Equivalent to operatorJoystick.povDown() — auto aim feed */
    public Trigger autoAimFeed() {
        return new Trigger(() -> getRawButton(AUTO_AIM_FEED));
    }

    /** Equivalent to operatorJoystick.back() — reset deploy position */
    public Trigger resetDeploy() {
        return new Trigger(() -> getRawButton(RESET_DEPLOY));
    }
}