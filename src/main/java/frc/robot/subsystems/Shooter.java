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
    
  private final SparkMax m_slave = new SparkMax(Constants.Shooter.ID_SHOOTER_BACK, MotorType.kBrushless); //Neo
  private final SparkMax m_master = new SparkMax(Constants.Shooter.ID_SHOOTER_FRONT, MotorType.kBrushless); //Neo
  private final SparkMax m_conveyor = new SparkMax(Constants.Shooter.ID_SHOOTER_CONVEYOR, MotorType.kBrushed); //Mini CIM
  private SparkMaxConfig slaveConfig = new SparkMaxConfig();
  private SparkMaxConfig masterConfig = new SparkMaxConfig();
  private SparkMaxConfig conveyorConfig = new SparkMaxConfig();
  private RelativeEncoder masterEncoder = m_master.getEncoder();
  private SparkClosedLoopController masterController = m_master.getClosedLoopController();
  private FeedForwardConfig feedForwardConfig = new FeedForwardConfig();
  private double masterTemp;
  private double slaveTemp;
  private int ready = 0;

  @SuppressWarnings("removal") // Reset y Safe Parameters de SparkConfig
  public Shooter() {
    // Idle Modes
    slaveConfig.idleMode(IdleMode.kCoast);
    masterConfig.idleMode(IdleMode.kCoast);
    conveyorConfig.idleMode(IdleMode.kBrake);

    // Current Limits
    conveyorConfig.smartCurrentLimit(Constants.Robot.DEFAULT_CURRENT, Constants.Robot.DEFAULT_CURRENT);
    masterConfig.smartCurrentLimit(Constants.Shooter.STALL_LIMIT, Constants.Shooter.FREE_LIMIT);
    slaveConfig.smartCurrentLimit(Constants.Shooter.STALL_LIMIT, Constants.Shooter.FREE_LIMIT);

    masterConfig.openLoopRampRate(0.5);
    masterConfig.closedLoopRampRate(0.5);
    slaveConfig.openLoopRampRate(0.5);
    slaveConfig.closedLoopRampRate(0.5);

    // Output Defaults
    slaveConfig.closedLoop.outputRange(-1, 1); //Defaults
    masterConfig.closedLoop.outputRange(-1, 1);
    slaveConfig.voltageCompensation(12);
    masterConfig.voltageCompensation(12);

    // FF Config
    feedForwardConfig.kV(Constants.Shooter.kV);
    masterConfig.closedLoop.feedForward.apply(feedForwardConfig);
    
    // PID Config  
    masterConfig.closedLoop
      .p(Constants.Shooter.kP)
      .i(Constants.Shooter.kI)
      .d(Constants.Shooter.kD);
    
    conveyorConfig.inverted(true);
    masterConfig.inverted(true);
    slaveConfig.inverted(false);

    // Master-Slave
    slaveConfig.follow(m_master, true);

    // Aplicar Configuraciones
    m_slave.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_master.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_conveyor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    masterTemp = m_master.getMotorTemperature();
    slaveTemp = m_slave.getMotorTemperature();
    updateSmartDashboard();
  }

  public void stopShooter(){
    m_master.stopMotor();
  }
  public void stopConveyor(){
    m_conveyor.stopMotor();
  }
  public void stopMotors(){
    stopShooter();
    stopConveyor();
  }
  public void setShooter(double speed){
    speed = MathUtil.clamp(speed, 0, Constants.Shooter.MAX_RPM);
    masterController.setSetpoint(speed, ControlType.kVelocity);
  }
  public void autoShoot(double speed){ //Debugging*
    setShooter(speed);
    autoConveyor();
  }
  public void autoConveyor() {
    ready = Math.abs(masterController.getSetpoint()-masterEncoder.getVelocity())<8?1 : 0;
    m_conveyor.set(Constants.Shooter.SHOOTER_CONVEYOR_SPEED*ready);
  }
  public void setConveyor() {
    m_conveyor.set(Constants.Shooter.SHOOTER_CONVEYOR_SPEED);
  }  
  public void updateSmartDashboard(){
    SmartDashboard.putNumber("Conveyor Output", m_conveyor.getAppliedOutput());
    SmartDashboard.putNumber("Back Output", m_slave.getAppliedOutput());
    SmartDashboard.putNumber("Front Output", m_master.getAppliedOutput());
    SmartDashboard.putNumber("Shooter Master Temp", masterTemp);
    SmartDashboard.putNumber("Shooter Slave Temp", slaveTemp);
    SmartDashboard.putBoolean("Shooter Overheating", masterTemp > Constants.Shooter.OVERHEAT_TEMP || slaveTemp > Constants.Shooter.OVERHEAT_TEMP);
    SmartDashboard.putNumber("Speed", masterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Error", Math.abs(masterEncoder.getVelocity()-masterController.getSetpoint()));
  }
    
  // ================COMMANDS&AUTOS===============
  public Command stopShooterCommand(){
    return Commands.runOnce(this::stopShooter, this);
  }
  public Command stopConveyorCommand(){
    return Commands.runOnce(this::stopConveyor, this);
  }
  public Command conveyorCommand(){
    return Commands.run(this::setConveyor, this);
  }

  // ===================TEST&DEPS==================
  public Command driveShooterCommand(double speed){
    return Commands.run(()-> this.setShooter(speed),this);
  }
  public SequentialCommandGroup shooterTestCommand(){
    return new SequentialCommandGroup(
      driveShooterCommand(1000),
      new WaitCommand(10),
      stopShooterCommand()
    );
  }
  public Command stopMotorsCommand(){
    return Commands.runOnce(this::stopMotors, this);
  }
  public Command shooterCommandClose(){
    return Commands.run(()-> this.setShooter(Constants.Shooter.NEO_SAFE_RPM*0.65),this);
  }
  public Command shooterCommandMid(){
    return Commands.run(()-> this.setShooter(Constants.Shooter.NEO_SAFE_RPM*0.75),this);
  }
  public Command shooterCommandFar(){
    return Commands.runOnce(()-> this.setShooter(Constants.Shooter.NEO_SAFE_RPM),this);
  }  
}
