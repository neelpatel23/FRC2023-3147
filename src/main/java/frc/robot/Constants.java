package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    // Driver Controller Ports (Xbox and Joysticks)
    public static final int kDriverControllerPort = 0;
    public static final int kUsbController1 = 1;
    public static final int kUsbController2 = 2;
  }
  public static class Motors {
    // Motor CANIDS
    public static final int CanID1 = 1;
    public static final int CanID2 = 2;
    public static final int CanID3 = 3;
    public static final int CanID4 = 4;
  }
  public static class ArmMotors {
    public static final int CANID5 = 5;
    public static final int CANID6 = 6;
    public static final int LIMIT = 310;
  }
  public static class ArmExtend {
    public static final int CANID7 = 7;
    public static final int LIMITEXTEND = -127;
    public static final int LIMITRETRACT = -5;
  }
  public static class HardwareCAN {
    // Hardware CANiDs
    public static final int PDU = 10;
    public static final int PneumaticHUB = 11;
  }
  public static class PneumaticChannels {
    public static final int FORWARD = 6;
    public static final int REVERSE = 7;
  }

  public static double DEADBAND = .2;

}
