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
    public static final int LIMIT = 620;
    public static final int LOWANGLE = 180;
    public static final int HIGHANGLE = 340;
  }
  public static class ArmExtend {
    public static final int CANID7 = 7;
    public static final int LIMITEXTEND = -155;
    public static final int LIMITRETRACT = -5;
    public static final int LIMITANGLE = -80;
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
  public static class SCORING_POSITIONS_ARM {
    public static final double TOP_NODE = 148;
  }
  public static class SCORING_POSITIONS_ER {
    public static final double TOP_NODE = -145;
    public static final double DEFAULT_POSITION = -5;
  }
  public static class PREFFEREDAUTO {
    public static final double MOVE_BACK_POINT1 = -77;
    public static final double MOVE_BACK_POINT2 = 77;
    public static final double MOVE_BACK_STOP_POINT1 = -72;
    public static final double MOVE_BACK_STOP_POINT2 = 72;
    public static final double MOVE_FORWARD_POINT1 = -38;
    public static final double MOVE_FORWARD_POINT2 = 38;
  }

  public static double DEADBAND = .2;

}
