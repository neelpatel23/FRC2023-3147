// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  // Controllers
  private XboxController controller = new XboxController(0);
  private XboxController controller1 = new XboxController(1);
  // Motor Controllers
  private CANSparkMax m_leftFront = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax m_leftRear = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax m_rightFront = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax m_rightRear = new CANSparkMax(4, MotorType.kBrushless);
  // Moter ControllerGroups (Right and Left)
  private MotorControllerGroup m_left = new MotorControllerGroup(m_leftFront, m_leftRear);
  private MotorControllerGroup m_right = new MotorControllerGroup(m_rightFront, m_rightRear);
  // Drive Mode
  private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  
  private double DriveXValue;
  private double DriveYValue;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_leftFront.setIdleMode(IdleMode.kCoast);
    m_leftRear.setIdleMode(IdleMode.kCoast);
    m_rightFront.setIdleMode(IdleMode.kCoast);
    m_rightRear.setIdleMode(IdleMode.kCoast);
    //m_leftFront.set(0.001);
    //m_leftRear.set(0.001);
    //m_rightFront.set(0.001);
    //m_rightRear.set(0.001);

    //m_left.setInverted(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {CommandScheduler.getInstance().run();}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Updating Dashboard Data
    SmartDashboard.putNumber("DriveXValue", DriveXValue);
    SmartDashboard.putNumber("Controller X", controller.getLeftX());
    SmartDashboard.putNumber("Controller Y", controller.getLeftY());
    SmartDashboard.putNumber("DriveXValue", DriveXValue);
    SmartDashboard.putNumber("DriveYValue", DriveYValue);
    // Deadbands and Controller
    DriveXValue = checkDeadband(controller.getLeftX());
    DriveYValue = checkDeadband(controller.getLeftY());
    // Network Tables (Info)
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
    NetworkTableEntry led = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode");
    NetworkTableEntry Z = NetworkTableInstance.getDefault().getTable("limelight").getEntry("Results");
    NetworkTableInstance.getDefault();
    SmartDashboard.putNumber("TV (Limelight)", tv);
    SmartDashboard.putNumber("TX (Limelight)", tx);
    SmartDashboard.putNumber("TY (Limelight)", ty);
    SmartDashboard.putNumber("TA (Limelight)", ta);
    System.out.print(Z);

    //10.31.47.11:5807 JSON Dump
    // double [] areas = Z.get();

    // System.out.print("areas: ");

    // for (double area : areas) {
    //   System.out.print(area + " ");
    // }
    // System.out.println();
    // System.out.print(JSON);
    // while ((tv == 1) && (ta > 5.0)) {
    //   if(ta > 5.0){
    //     m_drive.arcadeDrive(DriveXValue, 1.0);
    //   }
    //  m_drive.arcadeDrive(DriveXValue,   )
    // }
    // m_drive.arcadeDrive(DriveXValue, DriveYValue, true);
    m_drive.arcadeDrive(DriveXValue, DriveYValue, true);
    if (tv == 1) {
      led.setNumber(3);
    }
    else {
      led.setNumber(1);
    }
  }



  /** This function is called once when the robot is first started up. */
  public void Update_Limelight_Tracking() {
    final double STEER_K = 0.001;
    final double DRIVE_K = 0.26;
    final double DESIRED_TARGET_AREA = 6.50;
    final double MAX_DRIVE = 0.7;

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  
  }

  public double checkDeadband(double input)
  {
    if(input >= Constants.DEADBAND || input <= -Constants.DEADBAND)
    {
      return input;
    }
    
    return 0;
  }








  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {CommandScheduler.getInstance().cancelAll();}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
