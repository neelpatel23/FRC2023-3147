package frc.robot.systems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmExtend;
import frc.robot.Constants.ArmMotors;
import frc.robot.Constants.Motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ControlMotors {
    // Motors
    public CANSparkMax m_leftFront = new CANSparkMax(Motors.CanID2, MotorType.kBrushless);
    public CANSparkMax m_leftRear = new CANSparkMax(Motors.CanID4, MotorType.kBrushless);
    public CANSparkMax m_rightFront = new CANSparkMax(Motors.CanID1, MotorType.kBrushless);
    public CANSparkMax m_rightRear = new CANSparkMax(Motors.CanID3, MotorType.kBrushless);
    public CANSparkMax arm_left = new CANSparkMax(ArmMotors.CANID5, MotorType.kBrushless);
    public CANSparkMax arm_right = new CANSparkMax(ArmMotors.CANID6, MotorType.kBrushless);
    public CANSparkMax arm_extend = new CANSparkMax(ArmExtend.CANID7, MotorType.kBrushless);
    // Controller Groups
    public MotorControllerGroup m_left = new MotorControllerGroup(m_leftFront, m_leftRear);
    public MotorControllerGroup m_right = new MotorControllerGroup(m_rightFront, m_rightRear);
    public MotorControllerGroup arm_motors = new MotorControllerGroup(arm_left, arm_right);
    // Drive Type
    public DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
    // Constant Variables
    public double[] motorTemps = {0,0,0,0,0,0,0};

    public ControlMotors()
    {
        arm_left.setInverted(true);
    }

    public void Drive(double XDrive, double YDrive) {
        m_drive.setMaxOutput(SmartDashboard.getNumber("Drive Speed", 0));
        m_drive.arcadeDrive(XDrive, YDrive);
    }
    public void setBrakeIdle() {
        m_leftFront.setIdleMode(IdleMode.kBrake);
        m_leftRear.setIdleMode(IdleMode.kBrake);
        m_rightFront.setIdleMode(IdleMode.kBrake);
        m_rightRear.setIdleMode(IdleMode.kBrake);
    }
    public void setBrakeMode() {
        m_leftFront.setIdleMode(IdleMode.kBrake);
        m_leftRear.setIdleMode(IdleMode.kBrake);
        m_rightFront.setIdleMode(IdleMode.kBrake);
        m_rightRear.setIdleMode(IdleMode.kBrake);
    }
    public void setCoastMode() {
        m_leftFront.setIdleMode(IdleMode.kCoast);
        m_leftRear.setIdleMode(IdleMode.kCoast);
        m_rightFront.setIdleMode(IdleMode.kCoast);
        m_rightRear.setIdleMode(IdleMode.kCoast);
    }
    
    public void driveToZ() {
        m_drive.tankDrive(0.35, -0.35);
    } 
    public void driveBack() {
        
    }

    public void driveBackStraight(double Yaw) {
        if(Yaw < -.5)
        {
            m_drive.tankDrive(-0.4725, 0.5);
        }
        else if(Yaw > .5)
        {
            m_drive.tankDrive(-0.50, 0.4725);
        }
        else 
        {
            m_drive.tankDrive(-0.50, 0.5);
        }

    }

    public void driveForward() {
        m_drive.tankDrive(0.35, -0.35);
    }
    public void stopDrive() {
        m_drive.tankDrive(0, 0);
    }

    public void extendArm(double speed) {
        if(arm_extend.getEncoder().getPosition() >= Constants.ArmExtend.LIMITEXTEND && speed < 0)
        {
            arm_extend.set(speed);
        }

        else if(arm_extend.getEncoder().getPosition() <= Constants.ArmExtend.LIMITRETRACT && speed > 0)
        {
            arm_extend.set(speed);
        }
        
        else 
        {
            arm_extend.set(0);
        }
        
    }
    
    public void autoBalanceForward() {
        m_drive.tankDrive(0.30, -0.30);
    }
    public void autoBalanceBackward() {
        m_drive.tankDrive(-0.30, 0.30);
    }
    public void balanceDrive(double left, double right) {
        m_drive.tankDrive(left, right);
    }

    public void moveArm(double speed) 
    {
        if(arm_left.getEncoder().getPosition() <= Constants.ArmMotors.LIMIT && speed > 0)
        {
            arm_left.set(speed);
            arm_right.set(speed);
        }
        else if(arm_left.getEncoder().getPosition() >= -Constants.ArmMotors.LIMIT && speed < 0)
        {
            arm_left.set(speed);
            arm_right.set(speed);
        }
        else 
        {
            arm_left.set(0);
            arm_right.set(0);
        }
       
    }
    public void controlArm(double speed) {
        arm_extend.set(speed * SmartDashboard.getNumber("ExtendSpeed", speed));
    }

    public boolean isArmMoving() {
        return (arm_extend.get() == 0);
    }

    public boolean isDriveMoving() {
        return (m_rightFront.get() == 0);
    }

    private double toFarhenheit(double celsius) {
        return (celsius*1.8) + 32;
    }

    public void pushMotorTemps() {
        SmartDashboard.putNumber("Drv-LF-Temp", toFarhenheit(m_leftFront.getMotorTemperature()));
        SmartDashboard.putNumber("Drv-LR-Temp", toFarhenheit(m_leftRear.getMotorTemperature()));
        SmartDashboard.putNumber("Drv-RF-Temp", toFarhenheit(m_rightFront.getMotorTemperature()));
        SmartDashboard.putNumber("Drv-RR-Temp", toFarhenheit(m_rightRear.getMotorTemperature()));
        SmartDashboard.putNumber("Arm-L-Temp", toFarhenheit(arm_left.getMotorTemperature()));
        SmartDashboard.putNumber("Arm-R-Temp", toFarhenheit(arm_right.getMotorTemperature()));
        SmartDashboard.putNumber("Extend-Temp", toFarhenheit(arm_extend.getMotorTemperature()));
    }

    public void resetAllAutonEncoders() {
        m_leftRear.getEncoder().setPosition(0);
        m_rightFront.getEncoder().setPosition(0);
        arm_left.getEncoder().setPosition(0);
        arm_extend.getEncoder().setPosition(0);
    }

    public void resetDriveEncoders() {
        m_leftFront.getEncoder().setPosition(0);
        m_rightFront.getEncoder().setPosition(0);
    }

    public boolean isArmExtendedPastLowAngleLimit()
    {
        if(arm_extend.getEncoder().getPosition() < -80)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
    public boolean isArmExtendedToMaxLimit()
    {
        if(arm_extend.getEncoder().getPosition() < Constants.ArmExtend.LIMITEXTEND)
        {
            return true;
        }
        else
        {
            return false;
        }
    }


}
