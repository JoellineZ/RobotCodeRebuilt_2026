package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.PersistMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase {
  private final SparkMax m_arm = new SparkMax(Constants.Intake.ID_EXTDENDER, MotorType.kBrushless);
  private final SparkMax m_wheel = new SparkMax(Constants.Intake.ID_INTAKE_WHEEL, MotorType.kBrushless);   
  private SparkMaxConfig armConfig = new SparkMaxConfig();
  private SparkMaxConfig wheelConfig = new SparkMaxConfig();
  private SparkClosedLoopController armController = m_arm.getClosedLoopController();
  private final RelativeEncoder armEncoder= m_arm.getEncoder();
    
  public Intake() {
    armConfig.idleMode(IdleMode.kBrake);
    wheelConfig.idleMode(IdleMode.kBrake);
    armConfig.inverted(true);
    wheelConfig.inverted(false);
    armConfig.closedLoop
      .p(Constants.Intake.kP) // Los valores de PID son "Calibraciones" experimentales prueba y error. Recomendacion: Empieza con un valor que te de la funcion MAX_ERROR*kP = 1
      .i(Constants.Intake.kI)
      .d(Constants.Intake.kD);
    armConfig.closedLoopRampRate(0.25);
    armConfig.openLoopRampRate(0.25);
    wheelConfig.smartCurrentLimit(40);

    m_arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_wheel.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void drive(double drive_speed){
      if(Math.abs(drive_speed) > 1 ){
        drive_speed = drive_speed > 0 ? 1 : -1;
      }
      double motor_speed = drive_speed * Constants.Intake.MAX_ARM_SPEED_PERCENT;
      motor_speed =  stopAtLimit(motor_speed);

      m_arm.set(motor_speed);
  }

  private void setArmPosition(double position){
    position = MathUtil.clamp(position, Constants.Intake.MIN_ARM_POSITION, Constants.Intake.MAX_ARM_POSITION);
    armController.setSetpoint(position, ControlType.kPosition);
  }

  private void rollWheel(int direction) {
    m_wheel.set(armEncoder.getPosition()<5 ? 0 : direction * Constants.Intake.INTAKE_ROLL_SPEED);
  }
  private void stopWheel() {
    m_wheel.stopMotor();
  } 
  private void resetEncoders(){
    armEncoder.setPosition(0);
  }

  private void updateSmartDashboard(){
    SmartDashboard.putNumber("Arm Position", armEncoder.getPosition());
    SmartDashboard.putString("Arm Motor Temp", m_arm.getMotorTemperature()+"°C");
    SmartDashboard.putString("Roller Temp", m_wheel.getMotorTemperature()+"°C");
    SmartDashboard.putString("Roller Current", m_wheel.getOutputCurrent()+"A");
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  // ====================COMMANDS====================
  public Command driveCommand (XboxController controller, Intake m_intake){
    return Commands.run(
      ()->this.drive(
        controller.getRawAxis(Constants.IO.ID_JOYSTICK_SPEED)
        -controller.getRawAxis(Constants.IO.ID_JOYSTICK_BRAKE))
        ,this);
  }
  private double stopAtLimit(double input){
    double output = input;
    if (this.armEncoder.getPosition() <= 0 && input < 0){
        output = 0;
    }
    return output;
  }
  
  public Command rollWheelCommand(){
    return Commands.runOnce(()->this.rollWheel(1), this);
  }
  
  public Command rollBackWheelCommand(){
    return Commands.runOnce(()->this.rollWheel(-1), this);
  }
  public Command stopWheelCommand(){
    return Commands.runOnce(this::stopWheel, this);
  }
  public Command resetEncoderCommand(){
    return Commands.runOnce(this::resetEncoders, this);
  } 
  public Command extendCommand(){
    return Commands.run(()->this.setArmPosition(Constants.Intake.EXTENDED_POSITION), this).until(()->Math.abs(armEncoder.getPosition()-Constants.Intake.EXTENDED_POSITION)<Constants.Intake.MAX_ERROR);
  }
  public Command retractCommand(){
    return Commands.run(()->this.setArmPosition(Constants.Intake.RETRACTED_POSITION),this).until(()->Math.abs(armEncoder.getPosition()-Constants.Intake.RETRACTED_POSITION)<Constants.Intake.MAX_ERROR);
  }
}