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
  public static class HardwareCAN {
    // Hardware CANiDs
    public static final int PDU = 10;
    public static final int PneumaticHUB = 11;
  }

  public static double DEADBAND = .2;
}
