package frc.robot;

public class Constants {
  //Chassis
  public class Chassis {
    public static final int ID_CH_LF1 = 1;
    public static final int ID_CH_LF2 = 2;
    public static final int ID_CH_RG1 = 3;
    public static final int ID_CH_RG2 = 4;
    public static final int ID_ENC_CR1 = 0;
    public static final int ID_ENC_CR2 = 0;
    public static final int ID_ENC_CL1 = 0;
    public static final int ID_ENC_CL2 = 0;
    public static final double kRot = 1;
    public static final double kLoopRamp = 0.1;
    public static final double kDeadBandRot = 0.1;
    public static final double kPIDThreshold = 0.5;
    public static final double MAX_SPEED_ms2 = 3.0;
    public static final double MAX_ROTATION_SPEED_RAD_S = 1.0;
    public static final double Diameter = 12;
    public static final double kTrackWitdth = 0.6; //meters} 
  }

  public class Intake{
    public static final int ID_EXTD_NEO = 5;
    public static final double LIMIT_BWD = 0;
    public static final double LIMIT_FWD = 20;
  }

  public class IO{
    public static final int ID_CONTROLLER = 0;
  }
}