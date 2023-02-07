package frc.robot.systems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.hardware.Limelight;

public class AutoAlign {
    public DriveTrain autoDrive = new DriveTrain();
    public Limelight limeLight = new Limelight();
    public NetworkTableInstance inst;
    public NetworkTable ntLimelight;
    public NetworkTableEntry json;
    public NetworkTableEntry led;
    public String jsonin;
    public Double zDistance = 0.0;


    public void autoAlignDrive() {
    double tv = limeLight.getTV();
    double ta = limeLight.getTA();
    double tx = limeLight.getTX();
    double ty = limeLight.getTY();

    double z = getZDistance();
    do {
        // autoDrive.m_drive.tankDrive(0.10, -0.10, false);
        z = getZDistance();
    } while (z < -0.58);
}
    public Double getZDistance() {
    NetworkTable ntLimelight = inst.getTable("limelight");
    double tv = limeLight.getTV();
    led = ntLimelight.getEntry("ledMode");
    json = ntLimelight.getEntry("json");
    if(tv == 1 && json.isValid())
    {
        led.setNumber(3);
        
        if(json.getString("").indexOf("t6c_ts") > 0)
        {
        jsonin = json.getString("").substring(json.getString("").indexOf("t6c_ts"));
        jsonin = jsonin.substring(jsonin.indexOf("["), jsonin.indexOf("]"));
        jsonin = jsonin.replace("[","").replace("]", "");
        zDistance = Double.parseDouble(jsonin.split(",")[2]);
        SmartDashboard.putNumber("t6c_ts:z", zDistance);
        }
        else{
        jsonin = "";
        }
    }
    else
    {
        led.setNumber(1);
    }
    return zDistance;
    }
}
