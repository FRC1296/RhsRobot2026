package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {

    private TalonFX spindexerMotor;

    private DutyCycleOut dcOut = new DutyCycleOut(0);
    private VelocityVoltage velocityOut = new VelocityVoltage(0);

    private double statorCurrentLimit = 80.0;
    private StatusSignal spindexerVelocitySS;
    private StatusSignal spindexerVelocityErrorSS;
    private StatusSignal spindexerStatorCurrentSS;

    private BooleanPublisher spindexerStallPublisher;
    private SpindexerStallCommand stallCommand;

    public SpindexerSubsystem() {
        super("Spindexer");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable robotTable = inst.getTable(Constants.NETWORK_TABLE);
        NetworkTable spindexerTable = robotTable.getSubTable(Constants.NT_SPINDEXER);
        spindexerStallPublisher = spindexerTable.getBooleanTopic(Constants.NT_SPINDEXER_STALL).publish();
        spindexerStallPublisher.set(false);

        spindexerMotor = new TalonFX(Constants.feederConstants.SPINDEXER_MOTOR_ID);

        ConfigureSpindexerMotor();

        stallCommand = new SpindexerStallCommand(this);

    }

    private void ConfigureSpindexerMotor() {

        MotorOutputConfigs outputConfig = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(statorCurrentLimit);

        Slot0Configs slotZeroConfigs = new Slot0Configs()
                .withKG(0.0)
                .withKP(0.5)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.42)
                .withKV(0.1);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withMotorOutput(outputConfig)
                .withCurrentLimits(currentLimitConfig)
                .withSlot0(slotZeroConfigs);

        spindexerMotor.getConfigurator().apply(motorConfig);

        spindexerVelocitySS = spindexerMotor.getVelocity();
        spindexerVelocityErrorSS = spindexerMotor.getClosedLoopError();
        spindexerStatorCurrentSS = spindexerMotor.getStatorCurrent();
    }

    public void runSpindexer() {
        spindexerMotor.setControl(velocityOut.withSlot(0).withVelocity(Constants.feederConstants.SPINDEXER_SPEED));
    }

    public void reverseSpindexer() {
        spindexerMotor.setControl(velocityOut.withSlot(0).withVelocity(-20.0));
    }

    public void stopSpindexer() {
        spindexerMotor.setControl(dcOut.withOutput(0.0));
    }

    @Override
    public void periodic() {
        super.periodic();

        BaseStatusSignal.refreshAll(spindexerVelocitySS, spindexerVelocityErrorSS, spindexerStatorCurrentSS);

        boolean isStall = false;
        // Only check for stall when control is VelocityVoltage
        if (spindexerMotor.getAppliedControl() instanceof VelocityVoltage) {
            if (spindexerVelocityErrorSS.getValueAsDouble() > 11.0
                    && spindexerStatorCurrentSS.getValueAsDouble() > (statorCurrentLimit - 5.0)) {
                isStall = true;
                if (stallCommand.isScheduled() == false) {
                    CommandScheduler.getInstance().schedule(stallCommand);
                }  
            }
        }
        spindexerStallPublisher.set(isStall);
    }
}
