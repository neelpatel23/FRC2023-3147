package frc.robot.systems;

import edu.wpi.first.wpilibj.Encoder;
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

    public Encoder armEncoder = new Encoder(1, 2);


    // Constant Variables
    public double[] motorTemps = {0,0,0,0,0,0,0};
    public final double STEER_K = 0.03;                    // how hard to turn toward the target
    public final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
    public final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
    public final double MAX_DRIVE = 0.7;     

    public ControlMotors()
    {
        arm_left.setInverted(true);
        armEncoder.setReverseDirection(true);
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
    public void driveBackStraight(double Yaw) {
        m_drive.setMaxOutput(.9);
        if(Yaw < -.5)
        {
            m_drive.tankDrive(-0.4725, 0.50);
        }
        else if(Yaw > .5)
        {
            m_drive.tankDrive(-0.50, 0.4725);
        }
        else 
        {
            m_drive.tankDrive(-0.50, 0.50);
        }
    }
    public void driveForwardStraight(double Yaw) {
        m_drive.setMaxOutput(.9);
        if(Yaw < -.5){
            m_drive.tankDrive(0.60, -0.5725);
        }
        else if (Yaw > .5){
            m_drive.tankDrive(0.5725, -0.60);
        }
        else {
            m_drive.tankDrive(0.60,-0.60);
        }
    }

    public void driveForward() {
        m_drive.tankDrive(0.45, -0.45);
    }
    public void stopDrive() {
        m_drive.tankDrive(0, 0);
    }
    
    public void autoBalanceForward() {
        m_drive.tankDrive(0.15, -0.15, false);
    }
    public void autoBalanceBackward() {
        m_drive.tankDrive(-0.15, 0.15, false);
    }
    public void balanceDrive(double left, double right) {
        m_drive.tankDrive(left, right);
    }

    public void extendArm(double speed) {
        if(isArmExtendAllowed() && speed < 0)
        {
            arm_extend.set(speed);
        }

        else if(isArmRetractAllowed() && speed > 0)
        {
            arm_extend.set(speed);
        }
        
        else 
        {
            arm_extend.set(0);
        }
        
    }

    public void moveArm(double speed) 
    {
        if(isArmRotateForwardAllowed() && speed > 0)
        {
            arm_left.set(speed);
            arm_right.set(speed);
        }
        else if(isArmRotateReverseAllowed() && speed < 0)
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
        m_leftFront.getEncoder().setPosition(0);
        m_rightFront.getEncoder().setPosition(0);
        arm_left.getEncoder().setPosition(0);
        arm_extend.getEncoder().setPosition(0);
        armEncoder.reset();
    }

    public void resetDriveEncoders() {
        m_leftFront.getEncoder().setPosition(0);
        m_rightFront.getEncoder().setPosition(0);
    }

    public boolean isArmExtendAllowed()
    {
        //Stop arm from extending if outside of maximum limit
        if(arm_extend.getEncoder().getPosition() <= Constants.ArmExtend.LIMITEXTEND)
        {
            return false;
        }

        //Stop arm from extending if within the low angle limit (from zero). Prevents breaking the height restriction (6' 6")
        if(Math.abs(armEncoder.getDistance()) < Constants.ArmMotors.LOWANGLE && arm_extend.getEncoder().getPosition() < -80)
        {
            return false;
        }

        //Stop arm from extending if within the high angle limit (from zero). Prevents breaking the perimeter rule (4' 8")
        if(Math.abs(armEncoder.getDistance()) > Constants.ArmMotors.HIGHANGLE && arm_extend.getEncoder().getPosition() < -80)
        {
            return false;
        }

        return true;
    }

    public boolean isArmRetractAllowed()
    {
        //Stop arm from retracting if outside of minimum limit
        if(arm_extend.getEncoder().getPosition() >= Constants.ArmExtend.LIMITRETRACT)
        {
            return false;
        }

        return true;
    }

    public boolean isArmRotateForwardAllowed()
    {
        //Stop arm rotating outside of maximum limit
        if(armEncoder.getDistance() >= Constants.ArmMotors.LIMIT)
        {
            return false;
        }

        //Stop arm rotating forward if arm is extended past angle limit
        if(arm_extend.getEncoder().getPosition() < Constants.ArmExtend.LIMITANGLE)
        {
            //Check positive angle
            if(armEncoder.getDistance() > Constants.ArmMotors.HIGHANGLE)
            {
                return false;
            }

            //Check negative angle
            if(armEncoder.getDistance() > -Constants.ArmMotors.LOWANGLE && armEncoder.getDistance() < 0)
            {
                return false;
            }
        }

        return true;
    }

    public boolean isArmRotateReverseAllowed()
    {
        //Stop arm rotating outside of maximum limit
        if(armEncoder.getDistance() <= -Constants.ArmMotors.LIMIT)
        {
            return false;
        }

        //Stop arm rotating backward if arm is extended past angle limit
        if(arm_extend.getEncoder().getPosition() < Constants.ArmExtend.LIMITANGLE)
        {
            //Check positive angle
            if(armEncoder.getDistance() < -Constants.ArmMotors.HIGHANGLE)
            {
                return false;
            }

            //Check negative angle
            if(armEncoder.getDistance() < Constants.ArmMotors.LOWANGLE && armEncoder.getDistance() > 0)
            {
                return false;
            }
        }

        return true;
    }

    public boolean moveArmTo(double Rotate, double Extend)
    {
        double armEncoderValue = armEncoder.getDistance();
        double armExtend = arm_extend.getEncoder().getPosition();
        double rotateSpeed = 0.5;
        double extendSpeed = 0.75;
        SmartDashboard.putNumber("ArmEncoder2", armEncoderValue);
        SmartDashboard.putNumber(("Rotate"), Rotate);

        if((armEncoderValue < Rotate + 5) && (armEncoderValue > Rotate - 5) && (armExtend < Extend + 1) && (armExtend > Extend - 1))
        {
            //Arm in position
            moveArm(0);
            extendArm(0);
            return true;
        }

        if(Math.abs(Rotate - armEncoderValue) < 35)
        {
            rotateSpeed = 0.15;
        }

        if(Math.abs(Extend - armExtend) < 15)
        {
            extendSpeed = .35;
        }

        if(armEncoderValue > Rotate - 5)
        {
            moveArm(-rotateSpeed);
        }
        else if (armEncoderValue < Rotate + 5)
        {
            moveArm(rotateSpeed);
        }
        else {
            moveArm(0);
        }

        if(armExtend > Extend - 1)
        {
            extendArm(-extendSpeed);
        }
        else if(armExtend < Extend + 1)
        {
            extendArm(extendSpeed);
        }
        else {
            extendArm(0);
        }

        return false;
    }

}
