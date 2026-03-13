package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.units.measure.MutVoltage;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
  private final SparkMax m_back = new SparkMax(Constants.ID_SHOOTER_BACK, MotorType.kBrushless);
  private final SparkMax m_front = new SparkMax(Constants.ID_SHOOTER_FRONT, MotorType.kBrushless);
  private final SparkMax m_lineFuel = new SparkMax(Constants.ID_SHOOTER_LINE, MotorType.kBrushless);
  private SparkMaxConfig backConfig = new SparkMaxConfig();
  private SparkMaxConfig frontConfig = new SparkMaxConfig();
  private SparkMaxConfig lineFuelConfig = new SparkMaxConfig();

  public Shooter() {
    backConfig.idleMode(IdleMode.kCoast);
    frontConfig.idleMode(IdleMode.kCoast);
    lineFuelConfig.idleMode(IdleMode.kBrake);
    frontConfig.inverted(true);

    m_back.configure(backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_front.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_lineFuel.configure(lineFuelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  public void rollLineFuel() {
    m_lineFuel.set(Constants.SHOOTER_LINEFUEL_SPEED);
  }
  public void stopMotors(){
    m_back.stopMotor();
    m_front.stopMotor();
    m_lineFuel.stopMotor();
  }
    public void out(){
    m_back.set(0.5);
    m_front.set(0.5);
  }
  
  public Command stopShooterCommand(){
    return Commands.runOnce(this::stopMotors);
  }
  public Command lineFuelCommand(){
    return Commands.run(this::rollLineFuel);
  }
  public SequentialCommandGroup MainshooterCommand(){
    return new SequentialCommandGroup(
      Commands.runOnce(this::out),
      new WaitCommand(0.2),
      Commands.runOnce(this::lineFuelCommand)
    );
  }

}
