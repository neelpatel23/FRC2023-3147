package frc.robot;

// User Controls
import edu.wpi.first.wpilibj.Joystick;
// FRC
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OperatorConstants;
// Drive(s)
import frc.robot.systems.DriveTrain;
// Pneumatics
import frc.robot.systems.Pneumatics;
import frc.robot.systems.hardware.Limelight;
import frc.robot.systems.hardware.NavX;



public class Robot extends TimedRobot { 
  private DriveTrain drive = new DriveTrain();
  private Limelight limeLight = new Limelight();
  private Pneumatics pneumatics = new Pneumatics();
  private NavX navx = new NavX();
  public final XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Joystick Usb1 = new Joystick(OperatorConstants.kUsbController1);
  private Joystick Usb2 = new Joystick(OperatorConstants.kUsbController2);
  private double DriveXValue;
  private double DriveYValue;
  private double ArmSpeed;
  private double zDistance;
  private double updatedSteer;
  private double updatedDrive;
  private boolean JoystickControlEnable = true;

  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    drive.setBrakeIdle();
    pneumatics.enableCompressor();
    limeLight.initiateLimelight();

    SmartDashboard.putNumber("Yaw", navx.getYawMotion());
    SmartDashboard.putNumber("Pitch", navx.getPitchMotion());
    SmartDashboard.putNumber("Roll", navx.getRollMotion());
    SmartDashboard.putNumber("DriveXValue", DriveXValue);
    SmartDashboard.putNumber("DriveYValue", DriveYValue);
    SmartDashboard.putString("Drive Mode", "Brake");
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
    limeLight.updateLimelightDashboard();
    // Deadbands and Controller
    ArmSpeed = checkDeadband(controller.getRightY());
    DriveXValue = checkDeadband(controller.getLeftX());
    DriveYValue = checkDeadband(controller.getLeftY());
    SmartDashboard.putNumber("DriveXValue", DriveXValue);
    SmartDashboard.putNumber("DriveYValue", DriveYValue);
    SmartDashboard.putNumber("Yaw", navx.getYawMotion());
    SmartDashboard.putNumber("Pitch", navx.getPitchMotion());
    SmartDashboard.putNumber("Roll", navx.getRollMotion());
    /*
     * START OF ALL USER BASED CONTROLS
     */
    if (Usb1.getRawButtonPressed(1)) {
      // Driver Cam
      limeLight.setCurrentPipeline(0.0);
    }
    if (Usb1.getRawButtonPressed(2)) {
      // AprilTag
      limeLight.setCurrentPipeline(1.0);
    }
    if (Usb1.getRawButtonPressed(3)) {
      // Retro-Reflective
      limeLight.setCurrentPipeline(2.0);
    }
    if (Usb1.getRawButtonPressed(4)) {
      // Yellow Cone
      limeLight.setCurrentPipeline(3.0);
    }
    if (Usb1.getRawButtonPressed(5)) {
      // Purple Cube
      limeLight.setCurrentPipeline(4.0);
    }
    if (Usb2.getRawButtonPressed(1)) {
      // Set Motors to Brake
      drive.setBrakeMode();
      SmartDashboard.putString("Drive Mode", "Brake");
    }
    if (Usb2.getRawButtonPressed(2)) {
      // Set Motors to Coast
      drive.setCoastMode();
      SmartDashboard.putString("Drive Mode", "Coast");
    }
    if (Usb2.getRawButtonPressed(3)) {
      navx.getRollMotion();
      drive.balanceDrive(navx.autoBalancePositive()[0], navx.autoBalancePositive()[1]);
    }
    if (Usb2.getRawButtonPressed(4)) {
      double roll = navx.getRollMotion();
      boolean balanced = true;
      do(
        if((roll >= 0.0) && (roll <= 1.0)){
          balanced = false;
        }
        if(roll <= -1.0) {
          drive.autoBalanceForward();
        }
        else {
          drive.autoBalanceBackward();
        }
        roll = navx.getRollMotion();
      ) while (balanced);
      drive.Drive(0,0);
    }
    if(controller.getBButton()) {
      limeLight.Update_Limelight_Tracking();
      SmartDashboard.putNumber("DriveCom", limeLight.m_LimelightDriveCommand);
      SmartDashboard.putNumber("SteerCom", limeLight.m_LimelightSteerCommand);
      if (limeLight.m_LimelightHasValidTarget)
      {
        drive.Drive(limeLight.m_LimelightSteerCommand, limeLight.m_LimelightDriveCommand);
      }
      else
      {
        drive.Drive(0, 0);
      }
    }
    else
    {
      drive.Drive(DriveXValue, DriveYValue);
      drive.moveArm(ArmSpeed);
      SmartDashboard.putNumber("Arm", ArmSpeed);
    }
    /* ----------------------------------------------------------------------------------------------------- */
    if (controller.getAButton()) {
      JoystickControlEnable=false;
      zDistance = limeLight.getZDistance();
      if(zDistance < -0.58)
      //ZDistance < -0.58
      {
        //Drive
        drive.driveToZ();
      }
      else
      {
        drive.Drive(0, 0);
        controller.setRumble(RumbleType.kBothRumble,1);
      }
    }
    else
    {
      JoystickControlEnable = true;
      controller.setRumble(RumbleType.kBothRumble,0);
    }

    if (controller.getRightBumperPressed()) {
      pneumatics.openClaw();
    }
    if (controller.getLeftBumperPressed()) {
      pneumatics.closeClaw();
    }
    
    if(JoystickControlEnable) {drive.Drive(DriveXValue, DriveYValue);}
    

    SmartDashboard.putNumber("EncoderValue", drive.getEncoderValue());
    
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
