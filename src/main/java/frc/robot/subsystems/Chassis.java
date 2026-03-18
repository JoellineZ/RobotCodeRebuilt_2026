package frc.robot.subsystems;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class Chassis extends SubsystemBase {
  private final WPI_TalonSRX left1 = new WPI_TalonSRX(Constants.Chassis.ID_CH_LF1);
  private final WPI_TalonSRX left2 = new WPI_TalonSRX(Constants.Chassis.ID_CH_LF2);
  private final WPI_TalonSRX right1 = new WPI_TalonSRX(Constants.Chassis.ID_CH_RG1);
  private final WPI_TalonSRX right2 = new WPI_TalonSRX(Constants.Chassis.ID_CH_RG2);
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.Chassis.kTrackWidth);
  // private final DifferentialDrive m_drive = new DifferentialDrive(left1::set, right1::set);
  private Encoder l_encoder;
  private Encoder r_encoder;
  private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private boolean odometry_engaged;

  // Pathplanner
  private DifferentialDriveOdometry m_odometry;
  public double rot_monitor;

  public Chassis() {
    left2.follow(left1);
    right2.follow(right1);
    left1.setNeutralMode(NeutralMode.Brake);
    right1.setNeutralMode(NeutralMode.Brake);
    left2.setNeutralMode(NeutralMode.Brake);
    right2.setNeutralMode(NeutralMode.Brake);
    right1.setInverted(true);
    right2.setInverted(true);
    // left1.configOpenloopRamp(Constants.Chassis.kLoopRamp);
    // right1.configOpenloopRamp(Constants.Chassis.kLoopRamp);
    
    try{
      l_encoder = new Encoder(Constants.Chassis.ID_ENCODER_LEFT1, Constants.Chassis.ID_ENCODER_LEFT2);
      r_encoder = new Encoder(Constants.Chassis.ID_ENCODER_RIGHT1, Constants.Chassis.ID_ENCODER_RIGHT2);
      l_encoder.setDistancePerPulse(Constants.Chassis.ENCODER_TICK_RATIO);
      r_encoder.setDistancePerPulse(Constants.Chassis.ENCODER_TICK_RATIO);
      r_encoder.setReverseDirection(true);
      l_encoder.reset();
      r_encoder.reset();
      gyro.reset();
    }catch(Exception e){
      System.out.println("Gyro or Encoders not found, Odometry disabled");
    }
    initOdometry();
  }

  private void initOdometry(){
    // FOR GPIO/DIO use try catch to avoid failure on robot Init
    try{ 
      m_odometry = new DifferentialDriveOdometry(
          gyro.getRotation2d(), 
          l_encoder.getDistance(), 
          r_encoder.getDistance()
        );
      double previusTime = Timer.getFPGATimestamp();
      double previusTicks = l_encoder.get();
      double realTime = Timer.getFPGATimestamp();
      double realTicks = l_encoder.get();
      double dx = (realTicks - previusTicks) * Math.PI*Constants.Chassis.WHEEL_DIAMETER_INCHES;
      double dt = realTime - previusTime;
      
      // =========WPI==========
      // double velocity = dx/dt;
      // double distance = dx;
      odometry_engaged = true;
    } catch(Exception e){
      odometry_engaged=false;
    }finally{
      SmartDashboard.putBoolean("Encoder Stat", odometry_engaged);
    }
  }

  public void drive(ChassisSpeeds chassisSpeeds){
    var wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(Constants.Chassis.MAX_SPEED_ms);
    double leftPercent = wheelSpeeds.leftMetersPerSecond/Constants.Chassis.MAX_SPEED_ms;
    double rightPercent = wheelSpeeds.rightMetersPerSecond/Constants.Chassis.MAX_SPEED_ms;
    left1.set(leftPercent);
    right1.set(rightPercent);
  }

  public void arcadeDrive(double speed, double rot){
        // deadbands
        rot = Math.abs(rot)>=Constants.Chassis.kDeadBandRot ? rot : 0;
        speed = Math.abs(speed) >=Constants.Chassis.kDeadBandSpeed ? speed :0;
        double forwardSpeed = speed*Constants.Chassis.MAX_SPEED_ms;
        double rotationSpeed = rot*Constants.Chassis.MAX_ROTATION_SPEED_RAD_S;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
        forwardSpeed, 
        0, // No lateral movement in Tank Chassis
        rotationSpeed
        );
        drive(chassisSpeeds);    
    }

  public boolean StopChassis(){
    left1.stopMotor();
    right1.stopMotor();
    // Followers stop automatically
    return true;
  }

  private void SmartDashboard(){
    SmartDashboard.putBoolean("Odometry Engaged", odometry_engaged);
    if (odometry_engaged){
      SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
      SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
      SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
      SmartDashboard.putNumber("Left Encoder Ticks", l_encoder.get());
      SmartDashboard.putNumber("Right Encoder Ticks", r_encoder.get());
    }
  }

  public void clearFaults(){
    left1.clearStickyFaults();
    left2.clearStickyFaults();
    right1.clearStickyFaults();
    right2.clearStickyFaults();
    System.out.println("Successfully Cleared Drivetrain controllers Sticky Faults");
  }

  @Override
  public void periodic() {
    SmartDashboard();
  }

  // ============================COMMANDS============================
  public Command clearFaultsCommand(){
    return this.runOnce(() -> this.clearFaults());
  }
  public Command stopCommand(){
    return this.runOnce(() -> this.StopChassis());
  }
  public Command driveCommand(XboxController controller, Chassis chassis){ 
    return Commands.run(
      () -> 
        this.arcadeDrive(
          (controller.getRawAxis(Constants.IO.ID_JOYSTICK_SPEED)- controller.getRawAxis(Constants.IO.ID_JOYSTICK_BRAKE)),
        (Math.abs(controller.getRawAxis(Constants.IO.ID_JOYSTICK_ROT)) > Constants.Chassis.kDeadBandRot ? controller.getRawAxis(Constants.IO.ID_JOYSTICK_ROT) : 0))
        ,this);
  }
}
