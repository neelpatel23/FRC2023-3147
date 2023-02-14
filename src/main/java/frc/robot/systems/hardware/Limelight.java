package frc.robot.systems.hardware;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Limelight {
    private NetworkTableInstance inst;
    private NetworkTable ntLimelight;
    public NetworkTableEntry json;
    public NetworkTableEntry led;
    public String jsonin;
    public Double zDistance = 0.0;
    public Double xDistance = 0.0;
    public boolean m_LimelightHasValidTarget;
    public double m_LimelightDriveCommand;
    public double m_LimelightSteerCommand;


    public void initiateLimelight()  {
        inst = NetworkTableInstance.getDefault();
        ntLimelight = inst.getTable("limelight");
    }

    public void updateLimelightDashboard() {
        SmartDashboard.putNumber("Valid Target", ntLimelight.getEntry("tv").getDouble(0.0));
        SmartDashboard.putNumber("Horizoncal Offset", ntLimelight.getEntry("tx").getDouble(0.0));
        SmartDashboard.putNumber("Vertical Offset", ntLimelight.getEntry("ty").getDouble(0.0));
        SmartDashboard.putNumber("Area of Target", ntLimelight.getEntry("ta").getDouble(0.0));
    }
    public double metersToInches(double dist){
        double distInInches = dist * 39.3701;
        return distInInches;
    }
    public double getTX() {
        double tx = ntLimelight.getEntry("tx").getDouble(0);
        if (tx > 0) {
            return tx;
        }
        else {
            return 0;
        }
    }
    public double getTV() {
        double tv = ntLimelight.getEntry("tv").getDouble(0);
        if (tv > 0) {
            return tv;
        }
        else {
            return 0;
        }
    }
    public double getTY() {
        double ty = ntLimelight.getEntry("ty").getDouble(0);
        if (ty > 0) {
            return ty;
        }
        else {
            return 0;
        }
    }
    public double getTA() {
        double ta = ntLimelight.getEntry("ta").getDouble(0);
        if (ta > 0) {
            return ta;
        }
        else {
            return 0;
        }
    }
    public void setCurrentPipeline(double pipeline) {
        ntLimelight.getEntry("pipeline").setNumber(pipeline);
    }

    public void Update_Limelight_Tracking()
    {
          // These numbers must be tuned for your Robot!  Be careful!
          final double STEER_K = 15;                    // how hard to turn toward the target
          final double DRIVE_K = 26;                    // how hard to drive fwd toward the target
          final double DESIRED_TARGET_AREA = 3.948;        // Area of the target when the robot reaches the wall
          final double MAX_DRIVE = 30;                   // Simple speed limit so we don't drive too fast
    
          if (getTV() < 1.0)
          {
            m_LimelightHasValidTarget = false;
            m_LimelightDriveCommand = 0.0;
            m_LimelightSteerCommand = 0.0;
            return;
          }
  
          m_LimelightHasValidTarget = true;
  
          // Start with proportional steering
          double steer_cmd = getTX() * STEER_K;
          m_LimelightSteerCommand = steer_cmd;
  
          // try to drive forward until the target area reaches our desired area
          double drive_cmd = (DESIRED_TARGET_AREA - getTA()) * DRIVE_K;
  
          // don't let the robot drive too fast into the goal
          if (drive_cmd > MAX_DRIVE)
          {
            drive_cmd = MAX_DRIVE;
          }
          m_LimelightDriveCommand = drive_cmd;
          return;
    }
    public double getZDistance() {
        zDistance = 0.0;
        NetworkTable ntLimelight = inst.getTable("limelight");
        double tv = getTV();
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
                SmartDashboard.putNumber("last_t6c_ts:z_value", zDistance);
            }
            else{
            jsonin = "";
            }
        }
        else
        {
            led.setNumber(1);
        }
        SmartDashboard.putNumber("zDistance", zDistance);
        return zDistance;
        }

}
