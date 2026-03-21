package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

public class Constants {
  //Chassis
  public class Chassis {
    public static final int ID_CH_LF1 = 1;
    public static final int ID_CH_LF2 = 2;
    public static final int ID_CH_RG1 = 3;
    public static final int ID_CH_RG2 = 4;
    public static final int ID_ENCODER_RIGHT1 = 1; // Encoders not yet connected to RIO, ID 0S are placeholders
    public static final int ID_ENCODER_RIGHT2 = 2;
    public static final int ID_ENCODER_LEFT1 = 3;
    public static final int ID_ENCODER_LEFT2 = 4;
    public static final double kRot = 1;
    public static final double kLoopRamp = 0.1;
    public static final double kDeadBandSpeed = 0.05;
    public static final double kDeadBandRot = 0.1;
    public static final double kPIDThreshold = 0.5;
    public static final double MAX_SPEED_ms = 3.0;
    public static final double MAX_ROTATION_SPEED_RAD_S = 3.0;
    public static final double kTrackWidth = 0.6; //meters
    public static final double WHEEL_DIAMETER_INCHES = 6.0; //inches
    public static final double ENCODER_TICK_RATIO = (Math.PI * Units.inchesToMeters(WHEEL_DIAMETER_INCHES))/128;
    public static final double kS = 0.1;
    public static final double kV = 0.2;
    public static final double kA = 0.3;
    public static final double kP = 1.0;
    public static final double kI = 0.1;
    public static final double kD = 0.1;
  }

  public class Intake{
    public static final int ID_EXTDENDER = 8;
    public static final double LIMIT_BWD = 0;
    public static final double LIMIT_FWD = 20;
    public static final int ID_INTAKE_WHEEL = 9;
    public static final double MAX_ARM_SPEED_PERCENT = 0.3;
    public static final double INTAKE_ROLL_SPEED = 0.3;
    public static final double kP = 0.1; // 
    public static final double kI = 0;
    public static final double kD = 0.01;
    public static final double MIN_ARM_POSITION = 0;
    public static final double MAX_ARM_POSITION = 12.4;
    public static final double MAX_ERROR =0.25; //Rotations
    public static final double EXTENDED_POSITION = 12.4;
    public static final double RETRACTED_POSITION = 0;
  }

  public class Shooter{        
    public static final int ID_SHOOTER_BACK = 6;
    public static final int ID_SHOOTER_FRONT = 5;
    public static final int ID_SHOOTER_CONVEYOR = 7;
    public static final double SHOOTER_LINEFUEL_SPEED = 0.5;
    public static final double OVERHEAT_TEMP = 60.0;
    public static final double K_BACKSPIN = 0.6;
    public static final double NEO_SAFE_RPM = 2400.0;
    public static final double MAX_RPM = NEO_SAFE_RPM;
    public static final double MAX_RPM_FRONT = NEO_SAFE_RPM/(2-K_BACKSPIN);
    public static final double kP_f = 0.00025;
    public static final double kI_f = 0;
    public static final double kD_f = 0.002;
    public static final double kV = 0.45/NEO_SAFE_RPM;
    public static final double kP_b = kP_f;
    public static final double kI_b = 0;
    public static final double kD_b = kD_f;
  }

  public class IO{
    public static final int ID_CONTROLLER = 0;
    public static final int ID_DRIVER_CHASSIS = 1;
    public static final int ID_DRIVER_MECH = 0;    
    public static final int ID_JOYSTICK_SPEED = XboxController.Axis.kLeftTrigger.value;
    public static final int ID_JOYSTICK_ROT = 0;
    public static final int ID_JOYSTICK_BRAKE = XboxController.Axis.kRightTrigger.value;
}
}