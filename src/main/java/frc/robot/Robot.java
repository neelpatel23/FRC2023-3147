package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// User Controls
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
// FRC
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// Constants
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.OperatorConstants;
// Drive(s)
import frc.robot.systems.ControlMotors;
// Pneumatics
import frc.robot.systems.Pneumatics;
// Limelight (Vision)
import frc.robot.systems.hardware.Limelight;
// NavX2
import frc.robot.systems.hardware.NavX;

public class Robot extends TimedRobot { 
  private PowerDistribution pdh = new PowerDistribution(HardwareCAN.PDU, ModuleType.kRev);
  private ControlMotors ControlMotors = new ControlMotors();
  private Limelight limeLight = new Limelight();
  private Pneumatics pneumatics = new Pneumatics();
  private NavX navx = new NavX();
  public final XboxController controller = new XboxController(OperatorConstants.kDriverControllerPort);
  public RobotContainer m_robotContainer;
  public AddressableLED robot_leds;
  public AddressableLEDBuffer robot_leds_buffer;
  private Joystick Usb1 = new Joystick(OperatorConstants.kUsbController1);
  private Joystick Usb2 = new Joystick(OperatorConstants.kUsbController2);
  private double DriveXValue;
  private double DriveYValue;
  private double DriveEncoder1;
  private double DriveEncoder2;
  private double HlArm;
  private double ERArm;
  private double ArmSpeed;
  private double ExtendArm;
  private double RetractArm;
  private double zDistance;
  private double arm;
  private double arm2;
  private boolean step0 = true;
  private boolean step1 = false;
  private boolean step2 = false;
  private boolean step3 = false;
  private boolean step4 = false; 
  private boolean step5 = false; 
  private boolean step6 = false; 
  private boolean step7 = false; 

  private boolean DriverControl = true;
  private boolean autoAligning = false;
  public boolean isBalanced = true;
  private static final String a0 = "Preffered Position";
  private static final String a1 = "Secondary Position";
  private String autonModeSelected;
  private final SendableChooser<String> auton_chooser = new SendableChooser<>();


  @Override
  public void robotInit() {
    pdh.clearStickyFaults();
    robot_leds = new AddressableLED(0);
    robot_leds_buffer = new AddressableLEDBuffer(135);
    robot_leds.setLength(robot_leds_buffer.getLength());
    robot_leds.setData(robot_leds_buffer);
    robot_leds.start();
    m_robotContainer = new RobotContainer();
    ControlMotors.setBrakeIdle();
    pneumatics.enableCompressor();
    limeLight.initiateLimelight();
    SmartDashboard.putNumber("Roll", navx.getRollMotion());
    SmartDashboard.putNumber("Pitch", navx.getPitchMotion());
    SmartDashboard.putNumber("Yaw", navx.getYawMotion());
    SmartDashboard.putNumber("Extend Speed", .3);
    SmartDashboard.putString("Drive Mode", "Brake");
    SmartDashboard.putString("Drive Control", "True");
    SmartDashboard.putNumber("Drive Speed", .90);
    SmartDashboard.putBoolean("LEDS", true);
    // AUTON MODES
    auton_chooser.setDefaultOption("Preferred Auto", a0);
    auton_chooser.addOption("Preferred Auto", a0);
    auton_chooser.addOption("Secondary Auto", a1);
    SmartDashboard.putData(auton_chooser);
    // ControlMotors.getMotorTemps();
  }

  @Override
  public void robotPeriodic() {
    // ControlMotors.pushMotorTemps();
    CommandScheduler.getInstance().run();
    pdh.setSwitchableChannel(SmartDashboard.getBoolean("LEDS", true));
    SmartDashboard.putNumber("Storage Pressure", pneumatics.ph2.getPressure(0));
    SmartDashboard.putNumber("Working Pressure", pneumatics.ph2.getPressure(1));
  }

  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // ControlMotors.getMotorTemps();
    limeLight.updateLimelightDashboard();
    // Deadbands and Controller
    ArmSpeed = checkDeadband(controller.getRightY());
    arm = checkDeadband(controller.getRightTriggerAxis());
    arm2 = checkDeadband(controller.getLeftTriggerAxis());
    DriveXValue = checkDeadband(controller.getLeftX());
    DriveYValue = checkDeadband(controller.getLeftY());
    DriveEncoder1 = ControlMotors.m_leftRear.getEncoder().getPosition();
    DriveEncoder2 = ControlMotors.m_rightFront.getEncoder().getPosition();
    SmartDashboard.putNumber("Arm Encoder", ControlMotors.arm_left.getEncoder().getPosition());
    SmartDashboard.putNumber("Drive Encoder1", DriveEncoder1);
    SmartDashboard.putNumber("Drive Encoder2", DriveEncoder2);
    SmartDashboard.putNumber("Ex & Rt Encoder", ControlMotors.arm_extend.getEncoder().getPosition());
    ControlMotors.pushMotorTemps();

    SmartDashboard.putNumber("Roll", navx.getRollMotion());
    SmartDashboard.putNumber("Pitch", navx.getPitchMotion());
    SmartDashboard.putNumber("Yaw", navx.getYawMotion());

    /*
     * START OF ALL USER BASED CONTROLS
    */
    if (controller.getRightBumperPressed()) {pneumatics.closeClaw();}
    if (controller.getLeftBumperPressed()) {pneumatics.openClaw();}
    if (Usb1.getRawButtonPressed(1)) {/* Driver Cam */ limeLight.setCurrentPipeline(0);}
    if (Usb1.getRawButtonPressed(2)) {/* April Tag */ limeLight.setCurrentPipeline(1);}
    if (Usb1.getRawButtonPressed(3)) {/* Retro-Reflective */ limeLight.setCurrentPipeline(2);}
    if (Usb1.getRawButtonPressed(4)) {/* Yellow Cone */ limeLight.setCurrentPipeline(3);}
    if (Usb1.getRawButtonPressed(5)) {/* Purple Cube */ limeLight.setCurrentPipeline(4);}
    if (Usb2.getRawButtonPressed(1)) {ControlMotors.setBrakeMode() ; SmartDashboard.putString("Drive Mode", "Brake");}
    if (Usb2.getRawButtonPressed(2)) {ControlMotors.setCoastMode(); SmartDashboard.putString("Drive Mode", "Coast");}
    if (Usb2.getRawButtonPressed(7)) {
      for (var i = 0; i < robot_leds_buffer.getLength(); i++) {
        robot_leds_buffer.setHSV(i, 0, 255, 255);
      }
      robot_leds.setData(robot_leds_buffer);
    }
    if (Usb2.getRawButtonPressed(8)) {
      for (var i = 0; i < robot_leds_buffer.getLength(); i++) {
        robot_leds_buffer.setHSV(i, 240, 100, 54);
      }
      robot_leds.setData(robot_leds_buffer);
    }
    if (Usb2.getRawButton(9)) {
      rainbow();
      robot_leds.setData(robot_leds_buffer);
    }
    if (Usb2.getRawButtonReleased(9)) {
      for (var i = 0; i < robot_leds_buffer.getLength(); i++) {
        robot_leds_buffer.setRGB(i, 0, 0, 0);
      }
      robot_leds.setData(robot_leds_buffer);
    }
    /* ----------------------------------------------------------------------------------------------------- */
    if (Usb2.getRawButtonPressed(3)) {
      autoAligning = true;
    }
    if (autoAligning) {
      DriverControl = false;
      SmartDashboard.putString("Driver Control", "False");
      zDistance = limeLight.getZDistance();
      if(zDistance < -0.58)
      {
        ControlMotors.driveToZ();
      }
      else
      { 
        ControlMotors.Drive(0, 0);
        DriverControl = true;
        SmartDashboard.putString("Driver Control", "True");
        autoAligning = false;
        // controller.setRumble(RumbleType.kBothRumble, 1);
      }
    }
    if (Usb2.getRawButtonPressed(4)) {
      autoAligning = false;
      SmartDashboard.putString("Driver Control", "True");
      DriverControl = true;
    }
    if (Usb2.getRawButtonPressed(5)) {
      isBalanced = false;
      SmartDashboard.putString("Driver Control", "False");
      DriverControl = false;
    }
    if (isBalanced = false) {
      double roll = navx.getRollMotion();
      SmartDashboard.putNumber("Roll", navx.getRollMotion());
      if(roll <= -0.80) 
      {
        ControlMotors.autoBalanceForward();
      }
      else if (roll >= 0.80)
      {
        ControlMotors.autoBalanceBackward();
      } 
      roll = navx.getRollMotion();
      if ((roll <= 1.10) && (roll >= -1.10)) 
      {
        isBalanced = true;
        DriverControl = true;
        SmartDashboard.putString("Driver Control", "True");
      }
    }

    if (Usb2.getRawButtonPressed(6)) {
      isBalanced = true;
      DriverControl = true;
    }
    // TEMPORARY //
    if (controller.getAButtonPressed()) {
      ControlMotors.resetAllAutonEncoders();
    }
    // TEMPORARY //

    if ((DriverControl)) 
    {
      if (arm2 == 0) {
        ControlMotors.extendArm(-arm);
      }
      else if (arm == 0) {
        ControlMotors.extendArm(arm2);
      }  
      SmartDashboard.putString("Driver Control", "True");
      //controller.setRumble(RumbleType.kBothRumble, 0);
      ControlMotors.Drive(DriveXValue, DriveYValue);
      ControlMotors.moveArm(ArmSpeed);
    }
  }
  
  @Override
  public void autonomousInit() {
    ControlMotors.resetAllAutonEncoders();
    autonModeSelected = auton_chooser.getSelected();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    DriveEncoder1 = ControlMotors.m_leftRear.getEncoder().getPosition();
    DriveEncoder2 = ControlMotors.m_rightFront.getEncoder().getPosition();
    HlArm = ControlMotors.arm_left.getEncoder().getPosition();
    ERArm = ControlMotors.arm_extend.getEncoder().getPosition();
    SmartDashboard.putNumber("Ex & Rt Encoder", ERArm);
    SmartDashboard.putNumber("Arm Encoder", HlArm);
    SmartDashboard.putBoolean("Step0", step0);
    SmartDashboard.putBoolean("Step1", step1);
    SmartDashboard.putBoolean("Step2", step2);
    SmartDashboard.putBoolean("Step3", step3);
    SmartDashboard.putBoolean("Step4", step4);
    SmartDashboard.putBoolean("Step5", step5);
    SmartDashboard.putBoolean("Step6", step6);
    SmartDashboard.putBoolean("Step7", step7);

    switch (autonModeSelected) {
      case a0:
      // Lower Arm to placing point
      if (HlArm != 159 && HlArm <= 159) {
        ControlMotors.moveArm(1);
      } else if (HlArm >= 159 && HlArm <= 161) {
        ControlMotors.moveArm(0);
        step1 = true;
      }
      // Extend Arm to dropping height
      if (step1 == true) {
        if (ERArm != -124 && ERArm >= -124) {
          ControlMotors.extendArm(-0.45);
        }
        else if (ERArm <= -124 && ERArm >= -127) {
          ControlMotors.extendArm(0);
          step2 = true;
        }
      }
      // Open Claw to drop Cone
      if (step2 == true) {
        pneumatics.openClaw();
        step3 = true;
      }
      // Retract Arm back to starting position
      if (step3 == true) {
        if (ERArm <= -125 && ERArm >= -130) {
          ControlMotors.extendArm(0.45);
        }
        else if (ERArm == 1 && ERArm >= -3) {
          ControlMotors.extendArm(0);
          step4 = true;
        }
      }
      // Start Driving back while lowering and reversing arm position
      if (step4 == true) {
        if (HlArm != -315 && HlArm >= -315) {
          ControlMotors.moveArm(-1);
        }
        else if (HlArm <= -315 && HlArm >= -318) {
          ControlMotors.moveArm(0);
        }
        if ((DriveEncoder1 != -86 && DriveEncoder1 >= -86) && (DriveEncoder2 != 86 && DriveEncoder2 <= 86)) {
          ControlMotors.driveBack();
        }
        else if ((DriveEncoder1 <= -85 && DriveEncoder1 >= -88) && (DriveEncoder2 >= 85 && DriveEncoder2 <= 88)) {
          ControlMotors.stopDrive();
          step5 = true;
        }
      }
      if (step5 == true) {
        if ((DriveEncoder1 != -36 && DriveEncoder1 <= -36) && (DriveEncoder2 != 36 && DriveEncoder2 >= 36)) {
          ControlMotors.driveForward();
        }
        else if ((DriveEncoder1 <= -35 && DriveEncoder1 >= -38) && (DriveEncoder2 >= 35 && DriveEncoder2 <= 38)) {
          ControlMotors.stopDrive();
        }
      }
      break;
      case a1:
        // Lower Arm to placing point
        if(step0)
        {
          if (HlArm <= 142.5) {
            ControlMotors.moveArm(1);
          } else {
            ControlMotors.moveArm(0);
            step0 = false;
            step1 = true;
          }
        }
        
        // Extend Arm to dropping height
        if (step1) {
          if (ERArm >= -125) {
            ControlMotors.extendArm(-0.45);
          }
          else {
            ControlMotors.extendArm(0);
            step1 = false;
            step2 = true;
          }
        }

        // Open Claw to drop Cone
        if (step2) {
          pneumatics.openClaw();
          step2 = false;
          step3 = true;
        }

        // Retract Arm back to starting position
        if (step3) {
          if (ERArm <= -5) {
            ControlMotors.extendArm(0.45);
          }
          else {
            ControlMotors.extendArm(0);
            step3 = false;
            navx.ahrs.reset();
            step4 = true;
          }
        }
        // Start Driving back while lowering and reversing arm position
        if (step4) {
          if ((DriveEncoder1 >= -78) && (DriveEncoder2 <= 78)) {
            //ControlMotors.driveBack();
            SmartDashboard.putNumber("Yaw", navx.getYawMotion());
            ControlMotors.driveBackStraight(navx.getYawMotion());
          }
          else if ((DriveEncoder1 <= -78) && (DriveEncoder2 >= 78)) {
            ControlMotors.Drive(0, 0);
          }
          
          if (HlArm >= -315) {
            ControlMotors.moveArm(-1);
          }
          else if (HlArm <= -315) {
            ControlMotors.moveArm(0);
          }

          if (ControlMotors.isArmMoving() == false && ControlMotors.isDriveMoving() == false);
        }
    }
  }


  private int firsthue = 0;

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < robot_leds_buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (firsthue + (i * 180 / robot_leds_buffer.getLength())) % 180;
      // Set the value
      robot_leds_buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    firsthue += 3;
    // Check bounds
    firsthue %= 180;
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
