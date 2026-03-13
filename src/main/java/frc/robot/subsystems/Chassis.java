package frc.robot.subsystems;
//import static edu.wpi.first.units.Units.Meters;
//import static edu.wpi.first.units.Units.MetersPerSecond;
//import static edu.wpi.first.units.Units.Radians;
//import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.config.ModuleConfig;
//import com.pathplanner.lib.config.RobotConfig;
//import com.pathplanner.lib.controllers.PPLTVController;

public class Chassis extends SubsystemBase {
  private final WPI_TalonSRX left1 = new WPI_TalonSRX(Constants.ID_CH_LF1);
  private final WPI_TalonSRX left2 = new WPI_TalonSRX(Constants.ID_CH_LF2);
  private final WPI_TalonSRX right1 = new WPI_TalonSRX(Constants.ID_CH_RG1);
  private final WPI_TalonSRX right2 = new WPI_TalonSRX(Constants.ID_CH_RG2);
  private final DifferentialDrive m_drive = new DifferentialDrive(left1::set, right1::set);
    private Encoder l_encoder;
    private Encoder r_encoder;
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(); 
    private boolean odometry_engaged;
    private DifferentialDriveOdometry m_odometry;
    private static final double encoder_dpp = (Math.PI * Units.inchesToMeters(6))/128;
  public Chassis() {
    left2.follow(left1);
    right2.follow(right1);
    left1.setNeutralMode(NeutralMode.Brake);
    right1.setNeutralMode(NeutralMode.Brake);
    left2.setNeutralMode(NeutralMode.Brake);
    right2.setNeutralMode(NeutralMode.Brake);
    right1.setInverted(true);
    left1.configOpenloopRamp(Constants.kLoopRamp);
    right1.configOpenloopRamp(Constants.kLoopRamp);

    gyro.calibrate();
    m_odometry = new DifferentialDriveOdometry(
        gyro.getRotation2d(), 
        l_encoder.getDistance(), 
        r_encoder.getDistance()
      );
}
    private void initOdometry(){
    try{ //FOR GPIO/DIO use try catch to avoid failure on robot Init
      l_encoder = new Encoder(Constants.ID_ENC_CL1, Constants.ID_ENC_CL2);
      r_encoder = new Encoder(Constants.ID_ENC_CR1, Constants.ID_ENC_CR2);
      l_encoder.reset();
      r_encoder.reset();
      
      double previusTime = Timer.getFPGATimestamp();
      double previusTicks = l_encoder.get();
      double realTime = Timer.getFPGATimestamp();
      double realTicks = l_encoder.get();
      double dx = (realTicks - previusTicks) * Math.PI*Constants.Diameter;
      double dt = realTime - previusTime;
      double velocity = dx/dt;
      double distance = dx;
    
    

      odometry_engaged = true;
    } catch(Exception e){
      odometry_engaged=false;
    }finally{
      SmartDashboard.putBoolean("Encoder Stat", odometry_engaged);
    }
}
public double rot_monitor;
public void drive(ChassisSpeeds chassisSpeeds){
    double forwardXSpeed = chassisSpeeds.vxMetersPerSecond;
    if(Math.abs(forwardXSpeed)>Constants.MAX_SPEED_ms2){
        forwardXSpeed = forwardXSpeed > 0 ? Constants.MAX_SPEED_ms2 : -Constants.MAX_SPEED_ms2;
    }
    double angularVelocity = chassisSpeeds.omegaRadiansPerSecond;
    rot_monitor = angularVelocity;
    double leftVelocity = forwardXSpeed + (angularVelocity * Constants.kTrackWitdth / 2);
    double rightVelocity = forwardXSpeed - (angularVelocity * Constants.kTrackWitdth / 2);
    }
public void arcadeDrive(double speed, double rot){
        // deadbands
        rot = Math.abs(rot)>=0.001 ? rot : 0;
        speed = Math.abs(speed) >=0.001 ? speed :0;
        double forwardSpeed = speed*Constants.MAX_SPEED_ms2;
        double rotationSpeed = rot*Constants.MAX_ROTATION_SPEED_RAD_S;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
        forwardSpeed, 
        0, 
        rotationSpeed
        );
        drive(chassisSpeeds);

        
    }
    

  public Command driveCommand(XboxController controller){ 
        // SmartDashboard.putNumber("brake troubleshooting", playstationBrake);
    
    return Commands.run(
      () -> 
        this.arcadeDrive(
        (controller.getRawAxis(Constants.ID_JOYSTICK_SPEED)- controller.getRawAxis(Constants.ID_JOYSTICK_BRAKE)),
        (Math.abs(controller.getRawAxis(Constants.ID_JOYSTICK_ROT)) > Constants.kDeadBandRot ? controller.getRawAxis(Constants.ID_JOYSTICK_ROT) : 0)), 
      this);
  }

  public boolean StopChassis(){
    left1.stopMotor();
    right1.stopMotor();
    // Followers stop automatically
    return true;
  }
  
  public Command stopCommand(){
    return this.runOnce(() -> this.StopChassis());
  }

  public void clearFaults(){
    left1.clearStickyFaults();
    left2.clearStickyFaults();
    right1.clearStickyFaults();
    right2.clearStickyFaults();
    System.out.println("Successfully Cleared Drivetrain controllers Sticky Faults");
  }
  public Command clearFaultsCommand(){
    return this.runOnce(() -> this.clearFaults());
  }
}
