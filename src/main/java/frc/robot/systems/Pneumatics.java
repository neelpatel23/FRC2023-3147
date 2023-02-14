package frc.robot.systems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.PneumaticChannels;


public class Pneumatics {
    public Compressor comp = new Compressor(
        HardwareCAN.PneumaticHUB, 
        PneumaticsModuleType.REVPH
    );
    public DoubleSolenoid sold = new DoubleSolenoid(
        HardwareCAN.PneumaticHUB, 
        PneumaticsModuleType.REVPH, 
        PneumaticChannels.FORWARD, 
        PneumaticChannels.REVERSE
    );

    public void enableCompressor() {
        comp.enableDigital();
    }
    
    public void openClaw() {
        // comp.enableDigital();
        sold.set(Value.kForward);
    }
    public void closeClaw() {
        // comp.enableDigital();
        sold.set(Value.kReverse);
    }
      
}
