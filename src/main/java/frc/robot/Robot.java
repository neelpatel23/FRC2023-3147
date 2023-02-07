package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.OperatorConstants;
import frc.robot.systems.DriveTrain;
import frc.robot.systems.AutoAlign;
import edu.wpi.first.networktables.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.kauailabs.navx.frc.AHRS;



public class Robot extends TimedRobot { 
  private DriveTrain drive = new DriveTrain();
  // private AutoAlign align = new AutoAlign();
  private Compressor comp = new Compressor(HardwareCAN.PneumaticHUB, PneumaticsModuleType.REVPH);
  private DoubleSolenoid sold = new DoubleSolenoid(HardwareCAN.PneumaticHUB, PneumaticsModuleType.REVPH, 6, 7);
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);
  private Joystick Usb1 = new Joystick(OperatorConstants.kUsbController1);
  private Joystick Usb2 = new Joystick(OperatorConstants.kUsbController2);
  private double DriveXValue;
  private double DriveYValue;

  private AHRS ahrs = new AHRS(Port.kMXP);
  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    drive.m_leftFront.setIdleMode(IdleMode.kBrake);
    drive.m_leftRear.setIdleMode(IdleMode.kBrake);
    drive.m_rightFront.setIdleMode(IdleMode.kBrake);
    drive.m_rightRear.setIdleMode(IdleMode.kBrake);
    comp.enableDigital();
  }

  @Override
  public void robotPeriodic() {CommandScheduler.getInstance().run();}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
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
    SmartDashboard.putNumber("Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("Roll", ahrs.getRoll());
    // Updating Dashboard Data
    SmartDashboard.putNumber("DriveXValue", DriveXValue);
    SmartDashboard.putNumber("Controller X", controller.getLeftX());
    SmartDashboard.putNumber("Controller Y", controller.getLeftY());
    SmartDashboard.putNumber("DriveXValue", DriveXValue);
    SmartDashboard.putNumber("DriveYValue", DriveYValue);
    // Deadbands and Controller
    DriveXValue = checkDeadband(controller.getLeftX());
    DriveYValue = checkDeadband(controller.getLeftY());
    // System.out.print(Values);

   if (Usb2.getRawButtonPressed(1)) {
    System.out.print("Brake");
   }
   if (Usb2.getRawButtonPressed(2)) {
    System.out.print("Coast");
   }
   if (Usb1.getRawButtonPressed(1)) {
    System.out.print("forward");
    sold.set(Value.kForward);
   } 
   if (Usb1.getRawButtonPressed(2)) {
    sold.set(Value.kReverse);
   } 

    if (controller.getAButtonPressed()) {
      // drive.autoAlignDrive();
    }
    drive.Drive(DriveXValue, DriveYValue);
    
  }
  /** This function is called once when the robot is first started up. */
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
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
