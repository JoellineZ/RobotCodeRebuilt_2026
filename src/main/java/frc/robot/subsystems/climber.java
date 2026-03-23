package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climber extends SubsystemBase {
    private final SparkMax m_climberDown = new SparkMax(Constants.Climber.ID_CLIMBER_UP, MotorType.kBrushless);
    private final SparkMax m_climberUp = new SparkMax(Constants.Climber.ID_CLIMBER_DOWN, MotorType.kBrushless);   
    private final SparkMaxConfig m_climberConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_followerConfig = new SparkMaxConfig();

    @SuppressWarnings("removal")
    public climber(){
        m_climberConfig.idleMode(IdleMode.kBrake);
        m_climberConfig.closedLoop
            .p(Constants.Climber.kP)
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
        m_climberUp.set(climberSpeed* Constants.Climber.MAX_CLIMBER_SPEED);
    }
    private void stopClimber(){
        m_climberUp.stopMotor();
    }
    private void updateSmartDashboard(){
        SmartDashboard.putString("Climber Up Temp", m_climberUp.getMotorTemperature()+"oC");
        SmartDashboard.putString("Climber Down Temp", m_climberDown.getMotorTemperature()+"oC");
        SmartDashboard.putNumber("Climber Position", m_climberUp.getEncoder().getPosition());
    }
    @Override
    public void periodic() {
        updateSmartDashboard();
    }

    // =======================COMMANDS=======================
    public Command dumbClimberCommand(){
        return Commands.run(()->this.motorClimber(Constants.Climber.CLIMBER_SPEED), this); 
    }
    public Command dumbReverseClimberCommand(){
        return Commands.run(()->this.motorClimber(-Constants.Climber.CLIMBER_SPEED), this); 
    }
    public Command stopCommandClimberCommand(){
        return Commands.run(()->this.stopClimber(), this); 
    }
}
