package frc.robot.systems.hardware;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Limelight {
    private NetworkTableInstance inst;
    private NetworkTable ntLimelight;

    public void LimelightSubsystem()  {
        inst = NetworkTableInstance.getDefault();
        ntLimelight = inst.getTable("limelight");
        SmartDashboard.putNumber("TV (Limelight)", ntLimelight.getEntry("tv").getDouble(0.0));
        SmartDashboard.putNumber("TX (Limelight)", ntLimelight.getEntry("tx").getDouble(0.0));
        SmartDashboard.putNumber("TY (Limelight)", ntLimelight.getEntry("ty").getDouble(0.0));
        SmartDashboard.putNumber("TA (Limelight)", ntLimelight.getEntry("ta").getDouble(0.0));

    }
    // public double getLight() {
    //     NetworkTableEntry getLED = ntLimelight.getEntry("ledMode");
    //     return getLED();
    // }
   
    public double getTX() {
        double tx = ntLimelight.getEntry("tx").getDouble(0);
        return tx;
    }
    public double getTV() {
        double tv = ntLimelight.getEntry("tv").getDouble(0);
        return tv;
    }
    public double getTY() {
        double ty = ntLimelight.getEntry("ty").getDouble(0);
        return ty;
    }
    public double getTA() {
        double tx = ntLimelight.getEntry("ta").getDouble(0);
        return tx;
    }
    
}
