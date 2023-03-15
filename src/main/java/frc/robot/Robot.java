package frc.robot;

import java.time.Period;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// User Controls
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.cameraserver.CameraServer;
// FRC
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// Constants
import frc.robot.Constants.HardwareCAN;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PREFFEREDAUTO;
import frc.robot.Constants.SCORING_POSITIONS_ER;
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
  public AddressableLEDBuffer robot_leds_buffer_1;
  private Joystick Usb1 = new Joystick(OperatorConstants.kUsbController1);
  private Joystick Usb2 = new Joystick(OperatorConstants.kUsbController2);
  private double DriveXValue;
  private double DriveYValue;
  private double DriveEncoder1;
  private double DriveEncoder2;
  private double HlArm;
  private double ERArm;
  private double ArmSpeed;
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
  private boolean presettingArm = false;
  private boolean settingY = false;
  private boolean settingB = false;
  private boolean settingA = false;
  private boolean settingX = false;
  private int firsthue = 0;
  private boolean DriverControl = true;
  private boolean autoAligning = false;
  public boolean isBalanced = true;
  private static final String a0 = "Preffered Position";
  private static final String a1 = "Secondary Position";
  private String autonModeSelected;
  private final SendableChooser<String> auton_chooser = new SendableChooser<>();
  private double PeriodicCounter = 0;


  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    pdh.clearStickyFaults();
    robot_leds = new AddressableLED(0);
    robot_leds_buffer = new AddressableLEDBuffer(135);
    robot_leds.setLength(robot_leds_buffer.getLength());
    robot_leds.start();
    m_robotContainer = new RobotContainer();
    ControlMotors.setBrakeIdle();
    pneumatics.enableCompressor();
    limeLight.initiateLimelight();
    pneumatics.closeClaw();
    SmartDashboard.putNumber("Roll", navx.getRollMotion());
    SmartDashboard.putNumber("Pitch", navx.getPitchMotion());
    SmartDashboard.putNumber("Yaw", navx.getYawMotion());
    SmartDashboard.putNumber("Extend Speed", .3);
    SmartDashboard.putString("Drive Mode", "Brake");
    SmartDashboard.putString("Driver Control", "True");
    SmartDashboard.putNumber("Drive Speed", 0.80);
    SmartDashboard.putBoolean("LEDS", true);
    // AUTON MODES
    auton_chooser.setDefaultOption("Preferred Auto", a0);
    auton_chooser.addOption("Preferred Auto", a0);
    auton_chooser.addOption("Secondary Auto", a1);
    SmartDashboard.putData(auton_chooser);
    ControlMotors.resetAllAutonEncoders();
  }
  @Override
  public void disabledInit() {
    pneumatics.closeClaw();
    pdh.setSwitchableChannel(true);
  }
  @Override
  public void disabledPeriodic() {
    rainbow();
    robot_leds.setData(robot_leds_buffer);
  }

  @Override
  public void robotPeriodic() {
    // ControlMotors.pushMotorTemps();
    CommandScheduler.getInstance().run();
    pdh.setSwitchableChannel(SmartDashboard.getBoolean("LEDS", true));
    robot_leds.setData(robot_leds_buffer);
    SmartDashboard.putNumber("Storage Pressure", pneumatics.ph2.getPressure(0));
    SmartDashboard.putNumber("Working Pressure", pneumatics.ph2.getPressure(1));
  }

  @Override
  public void teleopInit() {
    for (var i = 0; i < robot_leds_buffer.getLength(); i++) {
      robot_leds_buffer.setRGB(i, 0, 0, 0);
    }
    robot_leds.setData(robot_leds_buffer);
    pdh.setSwitchableChannel(false);
    ControlMotors.m_drive.setMaxOutput(SmartDashboard.getNumber("Drive Speed", 0));

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    limeLight.updateLimelightDashboard();
    ControlMotors.pushMotorTemps();
    ArmSpeed = checkDeadband(controller.getRightY());
    arm = checkDeadband(controller.getRightTriggerAxis());
    arm2 = checkDeadband(controller.getLeftTriggerAxis());
    DriveXValue = checkDeadband(controller.getLeftX());
    DriveYValue = checkDeadband(controller.getLeftY());
    DriveEncoder1 = ControlMotors.m_leftFront.getEncoder().getPosition();
    DriveEncoder2 = ControlMotors.m_rightFront.getEncoder().getPosition();
    SmartDashboard.putNumber("Arm Encoder", ControlMotors.arm_left.getEncoder().getPosition());
    SmartDashboard.putNumber("Drive Encoder1", DriveEncoder1);
    SmartDashboard.putNumber("Drive Encoder2", DriveEncoder2);
    SmartDashboard.putNumber("Ex & Rt Encoder", ControlMotors.arm_extend.getEncoder().getPosition());
    SmartDashboard.putNumber("Roll", navx.getRollMotion());
    SmartDashboard.putNumber("Pitch", navx.getPitchMotion());
    SmartDashboard.putNumber("Yaw", navx.getYawMotion());
    SmartDashboard.putBoolean("IsArmRotateForwardAllowed", ControlMotors.isArmRotateForwardAllowed());
    SmartDashboard.putBoolean("IsArmRotateReverseAllowed", ControlMotors.isArmRotateReverseAllowed());
    SmartDashboard.putBoolean("IsArmExtendAllowed", ControlMotors.isArmExtendAllowed());
    SmartDashboard.putBoolean("IsArmRetractAllowed", ControlMotors.isArmRetractAllowed());

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
    // if (Usb2.getRawButtonPressed(3)) {
    //   autoAligning = true;
    // }
    // if (autoAligning) {
    //   DriverControl = false;
    //   SmartDashboard.putString("Driver Control", "False");
    //   zDistance = limeLight.getZDistance();
    //   if(zDistance < -0.58)
    //   {
    //     ControlMotors.driveToZ();
    //   }
    //   else
    //   { 
    //     ControlMotors.Drive(0, 0);
    //     DriverControl = true;
    //     SmartDashboard.putString("Driver Control", "True");
    //     autoAligning = false;
    //     // controller.setRumble(RumbleType.kBothRumble, 1);
    //   }
    // }
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
    if (isBalanced == false) {
      double roll = navx.getRollMotion();
      SmartDashboard.putNumber("Roll", navx.getRollMotion());
      if(roll <= -20.0) 
      {
        ControlMotors.autoBalanceForward();
      }
      else if (roll >= 20.0)
      {
        ControlMotors.autoBalanceBackward();
      }
      else if ((roll <= 20.0) && (roll >= -20.0)) 
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
    // if (controller.getAButtonPressed()) {
    //   ControlMotors.resetAllAutonEncoders();
    // }
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
      ControlMotors.Drive(DriveXValue, DriveYValue);
      ControlMotors.moveArm(ArmSpeed);
      if (controller.getYButtonReleased()) {presettingArm = false;}
      if (controller.getBButtonReleased()) {presettingArm = false;}
      if (controller.getAButtonReleased()) {presettingArm = false;}
      if (controller.getXButtonReleased()) {presettingArm = false;}
      settingY = controller.getYButton();
      settingB = controller.getBButton();
      settingA = controller.getAButton();
      settingX = controller.getXButton();
      if (settingY) {
        presetY();
      }
      if (settingB) {
        presetB();
      }
      if (settingA) {
        presetA();
      }
      if (settingX) {
        presetX();
      }

    }

    SmartDashboard.putNumber("ArmEncoder2", ControlMotors.armEncoder.getDistance());
  }

  public void presetY() {
    presettingArm = true;
    if (ControlMotors.moveArmTo(325, -155) && presettingArm) {
      presettingArm = false;
    }
  }
  public void presetB() {
    presettingArm = true;
    if (ControlMotors.moveArmTo(350, -71)) {
      presettingArm = false;
    }
  }
  public void presetA() {
    presettingArm = true;
    if (ControlMotors.moveArmTo(496, -2.5)) {
      presettingArm = false;
    }
  }
  public void presetX() {
    presettingArm = true;
    if (ControlMotors.moveArmTo(355, -53)) {
      presettingArm = false;
    }
  }
    
  @Override
  public void autonomousInit() { 
    autonModeSelected = auton_chooser.getSelected();
    for (var i = 0; i < robot_leds_buffer.getLength(); i++) {
      robot_leds_buffer.setRGB(i, 0, 0, 0);
    }
    robot_leds.setData(robot_leds_buffer);
    pdh.setSwitchableChannel(false);
    PeriodicCounter = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    DriveEncoder1 = ControlMotors.m_leftFront.getEncoder().getPosition();
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

    double roll = navx.getRollMotion();
    SmartDashboard.putNumber("Roll", navx.getRollMotion());

    switch (autonModeSelected) {
      case a0:
      if (step0) {
      // Lower Arm to placing point
        if(ControlMotors.moveArmTo(335, -155))
        {
          //Arm in position
          step0 = false;
          step1 = true;                   
        }
      }
      // Open Claw to drop Cone
      SmartDashboard.putNumber("PeriodicCounter", PeriodicCounter);
      if (step1) {
        PeriodicCounter += getPeriod();
        if(PeriodicCounter > 0.5) 
        {
          pneumatics.openClaw();
        }
        
        if(PeriodicCounter > 1.25)
        {
          step1 = false;
          step2 = true;
          step3 = true;
        }
      }
      // Retract Arm back to starting position
      if (step2) {
        if (ERArm <= SCORING_POSITIONS_ER.DEFAULT_POSITION) {
          ControlMotors.extendArm(0.85);
        }
        else {
          ControlMotors.extendArm(0);
          step2 = false;
          navx.ahrs.reset();
          step3 = true;
        }
      }
      // Start Driving back while lowering and reversing arm position
      if (step3) {
        if((DriveEncoder1 >= PREFFEREDAUTO.MOVE_BACK_POINT1) && (DriveEncoder2 <= PREFFEREDAUTO.MOVE_BACK_POINT2)) {
          SmartDashboard.putNumber("Yaw", navx.getYawMotion());
          ControlMotors.driveBackStraight(navx.getYawMotion());
        }
        else {
          ControlMotors.stopDrive();
          step3 = false;
          navx.ahrs.reset();
          step4 = true;
        }
      }
      if (step4) {
        ControlMotors.moveArm(0);
        if ((DriveEncoder1 <= PREFFEREDAUTO.MOVE_FORWARD_POINT1) && (DriveEncoder2 >= PREFFEREDAUTO.MOVE_FORWARD_POINT2)) {
          ControlMotors.driveForwardStraight(navx.getYawMotion());
        }
        //if(roll <= 15 || roll >= -15)
        //{
        //  ControlMotors.driveForwardStraight(navx.getYawMotion());
        //}
        else {
          ControlMotors.stopDrive();
          step4 = false;
          step5 = true;
        }
      }
      if (step5) {
        roll = navx.getRollMotion();
        SmartDashboard.putNumber("Roll", navx.getRollMotion());
        if(roll <= -10.0) 
        {
          ControlMotors.autoBalanceForward();
        }
        else if (roll >= 10.0)
        {
          ControlMotors.autoBalanceBackward();
        }
        else if ((roll <= 10.0) && (roll >= -10.0)) 
        {
          ControlMotors.stopDrive();
          // step6 = false;
        }
      }
      break;
      case a1:
        // Lower Arm to placing point
      if(step0) {
        if(ControlMotors.moveArmTo(335, -155))
        {
          //Arm in position
          step0 = false;
          step1 = true;                   
        }
      }
      // Extend Arm to dropping height
      if (step1) {
        PeriodicCounter += getPeriod();
        if(PeriodicCounter > 0.5) 
        {
          pneumatics.openClaw();
        }
        
        if(PeriodicCounter > 1.25)
        {
          step1 = false;
          step2 = true;
        }
      }
      // Open Claw to drop Cone
      if (step2) {
        if (ERArm <= SCORING_POSITIONS_ER.DEFAULT_POSITION) {
          ControlMotors.extendArm(0.85);
        }
        else {
          ControlMotors.extendArm(0);
          step2 = false;
          navx.ahrs.reset();
          step3 = true;
        }
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
        else {
          ControlMotors.stopDrive();
          step4 = false;
        }

        //if (ControlMotors.moveArmTo(-630, 0)) {
        // step4 = false;
        //}
        //if (ControlMotors.isArmMoving() == false && ControlMotors.isDriveMoving() == false);
      }
    }
  }

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
