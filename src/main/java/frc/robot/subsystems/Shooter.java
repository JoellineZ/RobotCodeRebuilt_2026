package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
  private final SparkMax m_back = new SparkMax(Constants.Shooter.ID_SHOOTER_BACK, MotorType.kBrushless); //Neo
  private final SparkMax m_front = new SparkMax(Constants.Shooter.ID_SHOOTER_FRONT, MotorType.kBrushless); //Neo
  private final SparkMax m_conveyor = new SparkMax(Constants.Shooter.ID_SHOOTER_CONVEYOR, MotorType.kBrushed); //Mini CIM
  private SparkMaxConfig motorConfig = new SparkMaxConfig();
  private SparkMaxConfig frontConfig = new SparkMaxConfig();
  private SparkMaxConfig conveyorConfig = new SparkMaxConfig();
  public SparkMaxConfig reConfig = new SparkMaxConfig();
  private RelativeEncoder backEncoder = m_back.getEncoder();
  private RelativeEncoder frontEncoder = m_front.getEncoder();
  private SparkClosedLoopController backController = m_back.getClosedLoopController();
  private SparkClosedLoopController frontController = m_front.getClosedLoopController();
  private FeedForwardConfig feedForwardConfig = new FeedForwardConfig();
  public double frontTemp;
  public double backTemp;

  @SuppressWarnings("removal") // Reset y Safe Parameters de SparkConfig
  public Shooter() {
    motorConfig.idleMode(IdleMode.kCoast);
    frontConfig.idleMode(IdleMode.kCoast);
    conveyorConfig.idleMode(IdleMode.kBrake);
    conveyorConfig.smartCurrentLimit(60, 80);
    motorConfig.smartCurrentLimit(60, 80);
    motorConfig.openLoopRampRate(0.5);
    motorConfig.closedLoopRampRate(0.5);
    motorConfig.closedLoop.outputRange(-1, 1);
    motorConfig.voltageCompensation(12);
    feedForwardConfig.kV(Constants.Shooter.kV);
    motorConfig.closedLoop.feedForward.apply(feedForwardConfig);
    
    frontConfig.smartCurrentLimit(60, 80);
    frontConfig.openLoopRampRate(0.5);
    frontConfig.closedLoopRampRate(0.5);
    frontConfig.closedLoop.outputRange(-1, 1);
    frontConfig.voltageCompensation(12);
    frontConfig.closedLoop.feedForward.apply(feedForwardConfig);
    
    motorConfig.closedLoop
      .p(Constants.Shooter.kP_b)
      .i(Constants.Shooter.kI_b)
      .d(Constants.Shooter.kD_b);
    
    frontConfig.closedLoop
      .p(Constants.Shooter.kP_f)
      .i(Constants.Shooter.kI_f)
      .d(Constants.Shooter.kD_f);
    
    conveyorConfig.inverted(true);
    frontConfig.inverted(true);
    // Aplicar Configuraciones
    m_back.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_front.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_conveyor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    frontTemp = m_front.getMotorTemperature();
    backTemp = m_back.getMotorTemperature();
    updateSmartDashboard();
  }

  public void stopMotors(){
    m_back.stopMotor();
    m_front.stopMotor();
    m_conveyor.stopMotor();
  }
  public void setShooter(double speed){
    speed = MathUtil.clamp(speed, 0, Constants.Shooter.MAX_RPM);
    double frontSpeed = speed;
    double backSpeed = speed;
    backController.setSetpoint(backSpeed, ControlType.kVelocity);
    frontController.setSetpoint(frontSpeed, ControlType.kVelocity);
    // TroubleShooting
    SmartDashboard.putNumber("Back Target", backSpeed);
    SmartDashboard.putNumber("Front Target", frontSpeed);
  }

  public void updateSmartDashboard(){
    SmartDashboard.putNumber("Conveyor Output", m_conveyor.getAppliedOutput());
    SmartDashboard.putNumber("Back Output", m_back.getAppliedOutput());
    SmartDashboard.putNumber("Front Output", m_front.getAppliedOutput());
    SmartDashboard.putNumber("Shooter Front Temp", frontTemp);
    SmartDashboard.putNumber("Shooter Back Temp", backTemp);
    SmartDashboard.putBoolean("Shooter Overheating", frontTemp > Constants.Shooter.OVERHEAT_TEMP || backTemp > Constants.Shooter.OVERHEAT_TEMP);
    SmartDashboard.putNumber("FrontSpeed", frontEncoder.getVelocity());
    SmartDashboard.putNumber("BackSpeed", backEncoder.getVelocity());
  }

  public void setConveyor() {
    m_conveyor.set(0.7);
  }
  
  // ====================COMMANDS====================
  public Command stopShooterCommand(){
    return Commands.runOnce(this::stopMotors, this);
  }
  public Command driveShooterCommand(double speed){
    return Commands.run(()-> this.setShooter(speed),this);
  }
  public SequentialCommandGroup shooterTestCommand(){
    return new SequentialCommandGroup(
      driveShooterCommand(500),
      new WaitCommand(10),
      stopShooterCommand()
    );
  }
  public Command shooterPIDCommandMid(){
    return Commands.run(()-> this.setShooter(2500),this);
  }

  public Command shooterPIDCommandClose(){
    return Commands.run(()-> this.setShooter(1500),this);
  }

  public Command shooterPIDCommandFar(){
    return Commands.run(()-> this.setShooter(3000),this);
  }

  public Command conveyorCommand(){
    return Commands.run(this::setConveyor, this);
  }
}
