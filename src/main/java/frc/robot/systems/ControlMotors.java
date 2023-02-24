package frc.robot.systems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public double[] motorTemps;


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
        m_drive.tankDrive(0.25, -0.25);
    } 
    public void driveBack() {
        m_drive.tankDrive(-0.50, 0.492);
    }
    public void driveForward() {
        m_drive.tankDrive(0.50, -0.50);
    }

    public void extendArm(double speed) {
        arm_extend.set(speed);
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

    public void moveArm(double speed) {
        arm_left.setInverted(true);
        arm_left.set(speed);
        arm_right.set(speed);
    }
    public void controlArm(double speed) {
        arm_extend.set(speed * SmartDashboard.getNumber("ExtendSpeed", speed));
    }

    // private double toFarhenheit(double celsius) {
    //     return (celsius*1.8) + 32;
    // }
    // public void getMotorTemps() {
    //     motorTemps[0] = toFarhenheit(m_leftFront.getMotorTemperature());
    //     motorTemps[1] = toFarhenheit(m_leftRear.getMotorTemperature());
    //     motorTemps[2] = toFarhenheit(m_rightFront.getMotorTemperature());
    //     motorTemps[3] = toFarhenheit(m_rightRear.getMotorTemperature());
    //     motorTemps[4] = toFarhenheit(arm_left.getMotorTemperature());
    //     motorTemps[5] = toFarhenheit(arm_right.getMotorTemperature());
    //     motorTemps[6] = toFarhenheit(arm_extend.getMotorTemperature());

    // }
    // public void pushMotorTemps() {
    //     SmartDashboard.putNumberArray("Motor Temps", motorTemps);
    // }
    public void resetAllAutonEncoders() {
        m_leftFront.getEncoder().setPosition(0);
        m_rightFront.getEncoder().setPosition(0);
        arm_left.getEncoder().setPosition(0);
        arm_extend.getEncoder().setPosition(0);
    }

}
