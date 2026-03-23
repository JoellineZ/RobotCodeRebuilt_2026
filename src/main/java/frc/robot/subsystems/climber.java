package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climber extends SubsystemBase {
    private final SparkMax m_climberDown = new SparkMax(Constants.Climber.ID_CLIMBER_UP, MotorType.kBrushless);
    private final SparkMax m_climberUp = new SparkMax(Constants.Climber.ID_CLIMBER_DOWN, MotorType.kBrushless);   
    private final SparkMaxConfig m_climberConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_followerConfig = new SparkMaxConfig();

    public climber(){
        m_climberConfig.idleMode(IdleMode.kBrake);
        m_climberConfig.closedLoop
      .p(Constants.Climber.kP) // Los valores de PID son "Calibraciones" experimentales prueba y error. Recomendacion: Empieza con un valor que te de la funcion MAX_ERROR*kP = 1
      .i(Constants.Climber.kI)
      .d(Constants.Climber.kD);
    m_climberConfig.closedLoopRampRate(0.25);
    m_climberConfig.openLoopRampRate(0.25);
    m_followerConfig.follow(m_climberUp);
    m_climberUp.configure(m_climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_climberDown.configure(m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    private void motorClimber(double climberSpeed){
        if(Math.abs(climberSpeed) > 1 ){
        climberSpeed = climberSpeed > 0 ? 1 : -1;
      }
      m_climberUp.set(climberSpeed* Constants.Climber.MAX_CLIMBER_SPEED
      );
    }
    private void stopClimber(){
        m_climberUp.stopMotor();
    }
    public Command dumbClimber(){
        return Commands.run(()->this.motorClimber(1), this); 
    }
    
    public Command dumbReverseClimber(){
        return Commands.run(()->this.motorClimber(-1), this); 
    }
    public Command stopCommandClimber(){
        return Commands.run(()->this.stopClimber(), this); 
    }
}
