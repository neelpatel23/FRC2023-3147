package frc.robot.systems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.ArmMotors;
import frc.robot.Constants.Motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveTrain {
    // Motors
    public CANSparkMax m_leftFront = new CANSparkMax(Motors.CanID2, MotorType.kBrushless);
    public CANSparkMax m_leftRear = new CANSparkMax(Motors.CanID4, MotorType.kBrushless);
    public CANSparkMax m_rightFront = new CANSparkMax(Motors.CanID1, MotorType.kBrushless);
    public CANSparkMax m_rightRear = new CANSparkMax(Motors.CanID3, MotorType.kBrushless);
    public CANSparkMax arm_left = new CANSparkMax(ArmMotors.CANID5, MotorType.kBrushless);
    public CANSparkMax arm_right = new CANSparkMax(ArmMotors.CANID6, MotorType.kBrushless);
    // Controller Groups
    public MotorControllerGroup m_left = new MotorControllerGroup(m_leftFront, m_leftRear);
    public MotorControllerGroup m_right = new MotorControllerGroup(m_rightFront, m_rightRear);
    public MotorControllerGroup arm_motors = new MotorControllerGroup(arm_left, arm_right);

    // Drive Type
    public DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
    // Constant Variables


    public void Drive(double XDrive, double YDrive) {
        m_drive.arcadeDrive((XDrive), (YDrive));
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
    public void balanceDrive(double left, double right) {
        m_drive.tankDrive(left, right);
    }

    public void moveArm(double speed) {
        arm_left.setInverted(true);
        arm_left.set(speed);
        arm_right.set(speed);
    }

    public double getEncoderValue() {
        return m_leftFront.getEncoder().getPosition();
    }
}
