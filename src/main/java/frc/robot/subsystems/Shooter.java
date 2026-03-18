package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
  private final SparkMax m_back = new SparkMax(Constants.Shooter.ID_SHOOTER_BACK, MotorType.kBrushless);
  private final SparkMax m_front = new SparkMax(Constants.Shooter.ID_SHOOTER_FRONT, MotorType.kBrushless);
  private final SparkMax m_lineFuel = new SparkMax(Constants.Shooter.ID_SHOOTER_LINE, MotorType.kBrushless);
  private SparkMaxConfig backConfig = new SparkMaxConfig();
  private SparkMaxConfig frontConfig = new SparkMaxConfig();
  private SparkMaxConfig lineFuelConfig = new SparkMaxConfig();
  public double frontTemp;
  public double backTemp;

  public Shooter() {
    backConfig.idleMode(IdleMode.kBrake);
    frontConfig.idleMode(IdleMode.kBrake);
    backConfig.smartCurrentLimit(35, 40);
    frontConfig.smartCurrentLimit(35, 40);
    lineFuelConfig.idleMode(IdleMode.kBrake);
    frontConfig.inverted(true);

    m_back.configure(backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_front.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_lineFuel.configure(lineFuelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    frontTemp = m_front.getMotorTemperature();
    backTemp = m_back.getMotorTemperature();
  }

  public void rollLineFuel() {
    m_lineFuel.set(Constants.Shooter.SHOOTER_LINEFUEL_SPEED);
  }
  public void stopMotors(){
    m_back.stopMotor();
    m_front.stopMotor();
    m_lineFuel.stopMotor();
  }
  public void driveShooter(double speed){
    speed = Math.max(-Constants.Shooter.MAX_RPM, Math.min(speed, Constants.Shooter.MAX_RPM)); // Clamp speed between 0 and 1
    m_back.set(speed*(Constants.Shooter.K_BACKSPIN));
  }
  public void updateSmartDashboard(){
    SmartDashboard.putNumber("Shooter Front Temp", frontTemp);
    SmartDashboard.putNumber("Shooter Back Temp", backTemp);
    SmartDashboard.putBoolean("Shooter Overheating", frontTemp > Constants.Shooter.OVERHEAT_TEMP || backTemp > Constants.Shooter.OVERHEAT_TEMP);
  }
  
  // ====================COMMANDS====================
  public Command stopShooterCommand(){
    return this.run(() -> stopMotors());
  }
  public Command lineFuelCommand(){
    return this.run(() -> rollLineFuel());
}
  public Command MainshooterCommand(double speed){
    return Commands.sequence(
        this.runOnce(() -> this.driveShooter(speed)),
        new WaitCommand(0.2),
        this.run(() -> this.rollLineFuel())
    );
}
}
