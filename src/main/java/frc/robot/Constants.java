package frc.robot;

public class Constants {
  //Chassis
  public class Chassis {
    public static final int ID_CH_LF1 = 1;
    public static final int ID_CH_LF2 = 2;
    public static final int ID_CH_RG1 = 3;
    public static final int ID_CH_RG2 = 4;
    //public static final int ID_ENC_CR1 = 0;
    //public static final int ID_ENC_CR2 = 0;
    //public static final int ID_ENC_CL1 = 0;
    //public static final int ID_ENC_CL2 = 0;
    public static final double kRot = 1;
    public static final double kLoopRamp = 0.1;
    public static final double kDeadBandRot = 0.1;
    public static final double kPIDThreshold = 0.5;
    public static final double MAX_SPEED_ms2 = 0.9;
    public static final double MAX_ROTATION_SPEED_RAD_S = 0.9;
    public static final double Diameter = 8;
    public static final double kTrackWitdth = 0.6; //meters
    public static final double WHEEL_DIAMETER_INCHES = 6.0;
  }

  public class Intake{
    public static final int ID_EXTD_NEO = 5;
    public static final double LIMIT_BWD = 1;
    public static final double LIMIT_FWD = 20;
    public static final double INTAKE_WHEEL_SPEED = 0;
    public static final int ID_INTAKE_ARM = 8;
    public static final int ID_INTAKE_WHEEL = 9;
    public static final double MAX_ARM_SPEED_PERCENT = 0;
    public static final double INTAKE_ROLL_SPEED = 0;
  }

  public class Shooter{
    public static final int ID_SHOOTER_BACK = 6;
    public static final int ID_SHOOTER_FRONT = 5;
    public static final int ID_SHOOTER_LINE = 7;
    public static final double SHOOTER_LINEFUEL_SPEED = 0.5;
    public static final double OVERHEAT_TEMP = 60.0;
    public static final double K_BACKSPIN = 0.4;
    public static final double NEO_SAFE_RPM = 3400;
    public static final double MAX_RPM = NEO_SAFE_RPM / (2-K_BACKSPIN);
  }

  public class Controller{
    public static final int ID_DRIVER_CHASSIS = 1;
    public static final int ID_DRIVER_MECH = 2;    
    public static final int ID_JOYSTICK_SPEED = 2;
    public static final int ID_JOYSTICK_ROT = 0;
    public static final int ID_JOYSTICK_BRAKE = 3;
  }
}