package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private Solenoid climberSolenoid;
    private Compressor climberCompressor;

    public ClimberSubsystem() {
        climberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        climberCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
        climberCompressor.enableDigital();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

    public void climberUp() {
        climberSolenoid.set(true);
    }

    public void climberdown() {
        climberSolenoid.set(false);
    }

}
