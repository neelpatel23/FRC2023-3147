package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.Motors;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.networktables.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;


public class Robot extends TimedRobot { 
  private Compressor comp = new Compressor(HardwareCAN.PneumaticHUB, PneumaticsModuleType.REVPH);
  private DoubleSolenoid sold = new DoubleSolenoid(HardwareCAN.PneumaticHUB, PneumaticsModuleType.REVPH, 6, 7);
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);
  private Joystick Usb1 = new Joystick(OperatorConstants.kUsbController1);
  private Joystick Usb2 = new Joystick(OperatorConstants.kUsbController2);
  private CANSparkMax m_leftFront = new CANSparkMax(Motors.CanID1, MotorType.kBrushless);
  private CANSparkMax m_leftRear = new CANSparkMax(Motors.CanID3, MotorType.kBrushless);
  private CANSparkMax m_rightFront = new CANSparkMax(Motors.CanID2, MotorType.kBrushless);
  private CANSparkMax m_rightRear = new CANSparkMax(Motors.CanID4, MotorType.kBrushless);
  private MotorControllerGroup m_left = new MotorControllerGroup(m_leftFront, m_leftRear);
  private MotorControllerGroup m_right = new MotorControllerGroup(m_rightFront, m_rightRear);
  private double DriveXValue;
  private double DriveYValue;
  private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  private AHRS ahrs = new AHRS(Port.kMXP);

  private NetworkTableInstance nti;
  double tv;
  double tx;
  double ty;
  double ta;
  NetworkTableEntry json;
  NetworkTableEntry led;
  String jsonin;
  Double zDistance = 0.0;

  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_leftFront.setIdleMode(IdleMode.kBrake);
    m_leftRear.setIdleMode(IdleMode.kBrake);
    m_rightFront.setIdleMode(IdleMode.kBrake);
    m_rightRear.setIdleMode(IdleMode.kBrake);
    nti = NetworkTableInstance.getDefault();
    comp.enableDigital();
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
    // Network Tables (Info)
    
    NetworkTable ntLimelight = nti.getTable("limelight");
    tv = ntLimelight.getEntry("tv").getDouble(0.0);
    tx = ntLimelight.getEntry("tx").getDouble(0.0);
    ty = ntLimelight.getEntry("ty").getDouble(0.0);
    ta = ntLimelight.getEntry("ta").getDouble(0.0);

    SmartDashboard.putNumber("TV (Limelight)", tv);
    SmartDashboard.putNumber("TX (Limelight)", tx);
    SmartDashboard.putNumber("TY (Limelight)", ty);
    SmartDashboard.putNumber("TA (Limelight)", ta);


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
    
    double z = getZDistance();
    do {
      m_drive.tankDrive(0.10, -0.10, false);
      z = getZDistance();
    } while (z < -0.58);

    m_drive.arcadeDrive(DriveXValue, DriveYValue, true);
    
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




  public Double getZDistance() {
    NetworkTable ntLimelight = nti.getTable("limelight");
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
