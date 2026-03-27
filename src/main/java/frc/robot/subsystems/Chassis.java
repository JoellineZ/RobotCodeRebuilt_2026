package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class Chassis extends SubsystemBase {
  private final WPI_TalonSRX left1 = new WPI_TalonSRX(Constants.Chassis.ID_CH_LF1);
  private final WPI_TalonSRX left2 = new WPI_TalonSRX(Constants.Chassis.ID_CH_LF2);
  private final WPI_TalonSRX right1 = new WPI_TalonSRX(Constants.Chassis.ID_CH_RG1);
  private final WPI_TalonSRX right2 = new WPI_TalonSRX(Constants.Chassis.ID_CH_RG2);
  private final PIDController anglePIDController = new PIDController(
    Constants.Chassis.kP_a, 
    Constants.Chassis.kI_a, 
    Constants.Chassis.kD_a);
  private final PIDController rightPIDController = new PIDController(
    Constants.Chassis.kP,
    Constants.Chassis.kI, 
    Constants.Chassis.kD
  );
  private final PIDController leftPIDController = new PIDController(
    Constants.Chassis.kP,
    Constants.Chassis.kI, 
    Constants.Chassis.kD
  );
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
    Constants.Chassis.kS, 
    Constants.Chassis.kV, 
    Constants.Chassis.kA
  );
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.Chassis.kTrackWidth);
  private Encoder l_encoder;
  private Encoder r_encoder;
  private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  
  // Pathplanner
  private boolean odometry_engaged=true;
  private DifferentialDriveOdometry m_odometry;
  private Field2d field = new Field2d();

  public Chassis() {
    left2.follow(left1);
    right2.follow(right1);
    left1.setNeutralMode(NeutralMode.Brake);
    right1.setNeutralMode(NeutralMode.Brake);
    left2.setNeutralMode(NeutralMode.Brake);
    right2.setNeutralMode(NeutralMode.Brake);
    right1.setInverted(true);
    right2.setInverted(true);

    l_encoder = new Encoder(Constants.Chassis.ID_ENCODER_LEFT1, Constants.Chassis.ID_ENCODER_LEFT2);
    r_encoder = new Encoder(Constants.Chassis.ID_ENCODER_RIGHT1, Constants.Chassis.ID_ENCODER_RIGHT2);
    l_encoder.setDistancePerPulse(Constants.Chassis.ENCODER_TICK_RATIO);
    r_encoder.setDistancePerPulse(Constants.Chassis.ENCODER_TICK_RATIO);
    
    l_encoder.reset();
    r_encoder.reset();
    l_encoder.setReverseDirection(false); // Inversion fisica de los encoders
    r_encoder.setReverseDirection(true);
    gyro.zeroYaw();
    gyro.reset();

    m_odometry = new DifferentialDriveOdometry(
      gyro.getRotation2d(), 
      l_encoder.getDistance(), 
      r_encoder.getDistance()
    );
    m_odometry.resetPosition(
      gyro.getRotation2d(), 
      l_encoder.getDistance(), 
      r_encoder.getDistance(), 
      new Pose2d()
    );

    // =====PATHPLANNER CONFIG=====
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e){
      e.printStackTrace();
      config = null;
    }

    if(config != null){
      AutoBuilder.configure(
        this::getPose,
        this::resetPose, 
        this::getRobotRelativeSpeeds, 
        (speeds, feedforwards)->drive(speeds), 
        new PPLTVController(0.02), 
        config, 
        ()-> {
          var alliance = DriverStation.getAlliance();
          if(alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
          } else {
            return false; // Default to blue if alliance information is not available 
          }
        },
        this);
    }
  }
  private void updateOdometry(){
    m_odometry.update(Rotation2d.fromDegrees(gyro.getYaw()), l_encoder.getDistance(),r_encoder.getDistance());
  }

  // ==========================METHODS=========================
  @Override
  public void periodic() {
    updateOdometry();
    field.setRobotPose(m_odometry.getPoseMeters());
    updateSmartDashboard();
  }

  @SuppressWarnings("unused")
  private void drive(ChassisSpeeds chassisSpeeds){
    var wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(Constants.Chassis.MAX_SPEED_ms);
    
    double leftFeedforward = m_feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightFeedforward = m_feedforward.calculate(wheelSpeeds.rightMetersPerSecond);

    double leftPID = leftPIDController.calculate(l_encoder.getRate(), wheelSpeeds.leftMetersPerSecond);
    double rightPID = rightPIDController.calculate(r_encoder.getRate(), wheelSpeeds.rightMetersPerSecond);

    // left1.set(leftFeedforward + leftPID);
    // right1.set(rightFeedforward + rightPID);
    right1.set(wheelSpeeds.rightMetersPerSecond/Constants.Chassis.MAX_SPEED_ms);
    left1.set(wheelSpeeds.leftMetersPerSecond/Constants.Chassis.MAX_SPEED_ms);
  }

  private void arcadeDrive(double speed, double rot){
    // if(speed<Constants.Chassis.kDeadBandRotSpeed&&Math.abs(rot)>=Constants.Chassis.kDeadBandRot){
    //   left1.set(rot*Constants.Chassis.K_AXIS_ROTATION);
    //   left2.set(-rot*Constants.Chassis.K_AXIS_ROTATION);
    // }else{
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
    // }    
  }

  private void StopChassis(){
    left1.stopMotor();
    right1.stopMotor();
  }

  @SuppressWarnings("unused")
  private void goToAngle(double target){
    anglePIDController.enableContinuousInput(-180, 180);
    double rotationOutput = anglePIDController.calculate(gyro.getYaw(), target);
    rotationOutput = MathUtil.clamp(rotationOutput, 
        -Constants.Chassis.MAX_ROTATION_SPEED_RAD_S, 
         Constants.Chassis.MAX_ROTATION_SPEED_RAD_S);
    drive(new ChassisSpeeds(0, 0, rotationOutput));
  }

  private void updateSmartDashboard(){
    SmartDashboard.putBoolean("Odometry Engaged", odometry_engaged);
    SmartDashboard.putNumber("Pose", gyro.getYaw());
    if (odometry_engaged){
      SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
      SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
      SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
      SmartDashboard.putNumber("Left Encoder Pos", l_encoder.getDistance());
      SmartDashboard.putNumber("Right Encoder Pos", r_encoder.getDistance());
      SmartDashboard.putData("Field",field);
    }
  }

  public void clearFaults(){
    left1.clearStickyFaults();
    left2.clearStickyFaults();
    right1.clearStickyFaults();
    right2.clearStickyFaults();
    System.out.println("Successfully Cleared Drivetrain controllers Sticky Faults");
  }

  // ============= PATHPLANNER METHODS =============
  public Pose2d getPose(){
    if(!odometry_engaged) return new Pose2d(); // Si no hay odometría devuelve un placeholder
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose){
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(gyro.getYaw()), 
      l_encoder.getDistance(), 
      r_encoder.getDistance(), 
      pose
    );
  }
  public void setTestPose(){
    Pose2d testPose = new Pose2d(l_encoder.getDistance(), 1, new Rotation2d(Math.toRadians(90)));
    resetPose(testPose);
    
  }

  public void resetOdometry(Pose2d pose) {
    l_encoder.reset();
    r_encoder.reset();
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getYaw()),
        l_encoder.getDistance(),
        r_encoder.getDistance(),
        pose
    );
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return m_kinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(
        l_encoder.getRate(),
        r_encoder.getRate())
      );
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
          (Math.abs(controller.getRawAxis(Constants.IO.ID_JOYSTICK_ROT)) > Constants.Chassis.kDeadBandRot ? -controller.getRawAxis(Constants.IO.ID_JOYSTICK_ROT) : 0))
          ,this
        );
  }
  public Command setTestPoseCommand(){
    return this.runOnce(() -> setTestPose());
  }

  // SYS ID -- [DELETE LATER] CODE FROM FRC DOCUMENTATION
  // Left
  private final MutVoltage m_leftVoltage = Volts.mutable(0);
  private final MutDistance m_leftDistance = Meters.mutable(0);
  private final MutLinearVelocity m_leftVelocity = MetersPerSecond.mutable(0);
  // Right
  private final MutVoltage m_rightVoltage = Volts.mutable(0);
  private final MutDistance m_rightDistance = Meters.mutable(0);
  private final MutLinearVelocity m_rightVelocity = MetersPerSecond.mutable(0);
  private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            voltage -> {
              left1.setVoltage(voltage);
              right1.setVoltage(voltage);
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism being
            // characterized.
            log -> {
              // Record a frame for the left motors.  Since these share an encoder, we consider
              // the entire group to be one motor.
              log.motor("drive-left")
                  .voltage(
                      m_leftVoltage.mut_replace(
                          left1.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_leftDistance.mut_replace(l_encoder.getDistance(), Meters))
                  .linearVelocity(
                      m_leftVelocity.mut_replace(l_encoder.getRate(), MetersPerSecond));
              // Record a frame for the right motors.  Since these share an encoder, we consider
              // the entire group to be one motor.
              log.motor("drive-right")
                  .voltage(
                      m_rightVoltage.mut_replace(
                          right1.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_rightDistance.mut_replace(r_encoder.getDistance(), Meters))
                  .linearVelocity(
                      m_rightVelocity.mut_replace(r_encoder.getRate(), MetersPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test state in
            // WPILog with this subsystem's name ("drive")
            this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}