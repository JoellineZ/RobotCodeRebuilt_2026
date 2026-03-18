package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  private RelativeEncoder backEncoder = m_back.getEncoder();
  private RelativeEncoder frontEncoder = m_front.getEncoder();
  private SparkClosedLoopController backController = m_back.getClosedLoopController();
  private SparkClosedLoopController frontController = m_front.getClosedLoopController();
  private FeedForwardConfig feedForwardConfig = new FeedForwardConfig();
  public double frontTemp;
  public double backTemp;

  @SuppressWarnings("removal") // Reset y Safe Parameters de SparkConfig
  public Shooter() {
    // Configurar shooter motor defaults y conveyor
    motorConfig.idleMode(IdleMode.kCoast);
    conveyorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(60, 80);
    motorConfig.closedLoop
      .p(Constants.Shooter.kP)
      .i(Constants.Shooter.kI)
      .d(Constants.Shooter.kD);
    motorConfig.openLoopRampRate(0.5);
    motorConfig.closedLoopRampRate(0.25);
    motorConfig.closedLoop.outputRange(0, 1);
    motorConfig.voltageCompensation(12);
    feedForwardConfig.kV(Constants.Shooter.kV);
    motorConfig.closedLoop.feedForward.apply(feedForwardConfig);

    // Motor front igual al back pero invertido
    frontConfig = motorConfig;
    frontConfig.inverted(true);
    conveyorConfig.inverted(true);
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
    speed = Math.max(0, Math.min(speed, Constants.Shooter.MAX_RPM_FRONT)); // Speed Limit
    double frontSpeed = speed;
    double backSpeed = speed*(2.0-Constants.Shooter.K_BACKSPIN);

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
    m_conveyor.set(0.8);
  }

  public void dumbShooterBackTest(){
    m_back.set(0.6);
  }

  public void dumbShooterFrontTest(){
    m_front.set(0.4);
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
  
  public Command conveyorCommand(){
    return Commands.run(this::setConveyor, this);
  }
  public Command dumbTestBackCommand(){
    return Commands.run(this::dumbShooterBackTest, this);
  }
  public Command dumbTestFrontCommand(){
    return Commands.run(this::dumbShooterFrontTest, this);
  }
}
