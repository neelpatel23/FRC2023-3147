package frc.robot.systems.hardware;

import edu.wpi.first.wpilibj.I2C.Port;
import com.kauailabs.navx.frc.AHRS;

public class NavX {
    public final AHRS ahrs = new AHRS(Port.kMXP);

    public double getYawMotion() {
        double yaw = ahrs.getYaw();
        // Rotation around the Z-Axis
        // -180 to 180
        return yaw;
    }
    public double getPitchMotion() {
        double pitch = ahrs.getPitch();
        // Rotation around the X-Axis
        // -180 to 180
        return pitch;
    }
    public double getRollMotion() {
        double roll = ahrs.getRoll();
        return roll;
    }

    public double[] autoBalancePositive() {
        // getRollMotion();
        double[] driveBackValues = new double[2];
        // Tank 1
        driveBackValues[0] = -0.25;
        // Tank 2
        driveBackValues[1] = 0.25;
        if (getRollMotion() > 10.0)
        {
            return driveBackValues;
        }
        else {
            return null;
        }
    }
    public double[] autoBalanceNegative() {
        // getRollMotion();
        double[] driveForwardValues = new double[2];
        // Tank 1
        driveForwardValues[0] = 0.25;
        // Tank 2
        driveForwardValues[1] = -0.25;
        if (getRollMotion() > -10.0)
        {
            return driveForwardValues;
        }
        else {
            return null;
        }
    }
}
