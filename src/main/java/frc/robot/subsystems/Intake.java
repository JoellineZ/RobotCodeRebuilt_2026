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
  private final SparkMax m_leftWheel = new SparkMax(Constants.Intake.ID_INTAKE_LFWHEEL, MotorType.kBrushed);
  private final SparkMax m_rightWheel = new SparkMax(Constants.Intake.ID_INTAKE_RFWHEEL, MotorType.kBrushed);   
  private SparkMaxConfig armConfig = new SparkMaxConfig();
  private SparkMaxConfig lwheelConfig = new SparkMaxConfig();
  private SparkMaxConfig rWheelConfig = new SparkMaxConfig();
  private SparkClosedLoopController armController = m_arm.getClosedLoopController();
  private final RelativeEncoder armEncoder= m_arm.getEncoder();
    
  public Intake() {
    armConfig.idleMode(IdleMode.kBrake);
    lwheelConfig.idleMode(IdleMode.kBrake);
    rWheelConfig.idleMode(IdleMode.kBrake);
    
    armConfig.inverted(true);
    armConfig.closedLoop
      .p(Constants.Intake.kP) // Los valores de PID son "Calibraciones" experimentales prueba y error. Recomendacion: Empieza con un valor que te de la funcion MAX_ERROR*kP = 1
      .i(Constants.Intake.kI)
      .d(Constants.Intake.kD);
    armConfig.closedLoop.positionWrappingInputRange(-1, 13.4);
    armConfig.closedLoop.positionWrappingEnabled(true);
    armConfig.closedLoopRampRate(0.25);
    armConfig.openLoopRampRate(0.25);

    lwheelConfig.inverted(false);
    rWheelConfig.inverted(true);
    lwheelConfig.smartCurrentLimit(40);
    rWheelConfig.smartCurrentLimit(40);

    m_arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftWheel.configure(lwheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightWheel.configure(rWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void drive(double drive_speed){
      drive_speed = MathUtil.clamp(drive_speed, -1, 1);
      double motor_speed = drive_speed * Constants.Intake.MAX_ARM_SPEED_PERCENT;
      motor_speed =  stopAtLimit(motor_speed);
      m_arm.set(motor_speed);
  }

  private void setArmPosition(double position){
    position = MathUtil.clamp(position, Constants.Intake.MIN_ARM_POSITION, Constants.Intake.MAX_ARM_POSITION);
    armController.setSetpoint(position, ControlType.kPosition);
  }

  private void rollWheel(int direction) {
    m_leftWheel.set(armEncoder.getPosition()<5 ? 0 : direction * Constants.Intake.INTAKE_ROLL_SPEED);
    m_rightWheel.set(armEncoder.getPosition()<5 ? 0 : direction * Constants.Intake.INTAKE_ROLL_SPEED);
  }
  private void stopWheel() {
    m_rightWheel.stopMotor();
    m_leftWheel.stopMotor();
  } 
  private void resetEncoders(){
    armEncoder.setPosition(0);
  }
  private double stopAtLimit(double input){
      double output = input;
      if ((this.armEncoder.getPosition() <= Constants.Intake.MIN_ARM_POSITION && input < 0) || (this.armEncoder.getPosition()>= Constants.Intake.MAX_ARM_POSITION && input>0)){
          output = 0;
      }
      return output;
  }
  private void updateSmartDashboard(){
    SmartDashboard.putNumber("Arm Position", armEncoder.getPosition());
    SmartDashboard.putString("Arm Motor Temp", m_arm.getMotorTemperature()+"oC");
    SmartDashboard.putString("Roller left Temp", m_leftWheel.getMotorTemperature()+"oC");
    SmartDashboard.putString("Roller right Temp", m_rightWheel.getMotorTemperature()+"oC");
    SmartDashboard.putString("Roller left Current", m_leftWheel.getOutputCurrent()+"A");
    SmartDashboard.putString("Roller right Current", m_rightWheel.getOutputCurrent()+"A");
    SmartDashboard.putNumber("Arm Setpoint Error", Math.abs(armController.getSetpoint()-armEncoder.getPosition()));

  }
  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  // ====================COMMANDS====================
  public Command driveCommand (XboxController controller){
    return Commands.run(
      ()->this.drive(
        controller.getRawAxis(Constants.IO.ID_JOYSTICK_SPEED)
        -controller.getRawAxis(Constants.IO.ID_JOYSTICK_BRAKE))
        ,this);
  }
  public Command rollWheelCommand(){
    return Commands.run(()->this.rollWheel(1), this);
  }
  public Command rollBackWheelCommand(){
    return Commands.run(()->this.rollWheel(-1), this);
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