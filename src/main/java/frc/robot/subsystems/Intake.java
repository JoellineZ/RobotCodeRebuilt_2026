package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
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
  private FeedForwardConfig armFeedForwardConfig = new FeedForwardConfig();
  private final RelativeEncoder armEncoder= m_arm.getEncoder();
    
  public Intake() {
    armConfig.idleMode(IdleMode.kBrake);
    wheelConfig.idleMode(IdleMode.kBrake);
    armConfig.inverted(false);
    wheelConfig.inverted(false);

    // Configure PID and FFWD
    armConfig.closedLoop
      .p(Constants.Intake.kP) // Los valores de PID son "Calibraciones" experimentales prueba y error. Recomendacion: Empieza con un valor que te de la funcion MAX_ERROR*kP = 1
      .i(Constants.Intake.kI)
      .d(Constants.Intake.kD);
    armFeedForwardConfig.kV(0); // Obtener con SysID
    armConfig.closedLoop.feedForward.apply(armFeedForwardConfig);

    m_arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_wheel.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void drive(double drive_speed){
      if(Math.abs(drive_speed) > 1 ){
        drive_speed = drive_speed > 0 ? 1 : -1;
      }
      double motor_speed = drive_speed * Constants.Intake.MAX_ARM_SPEED_PERCENT;
      motor_speed =  stopAtLimit(motor_speed);

      m_arm.set(motor_speed);
  }

  public void setArmPosition(double position){
    position = MathUtil.clamp(position, Constants.Intake.MIN_ARM_POSITION, Constants.Intake.MAX_ARM_POSITION);
    armController.setSetpoint(position, ControlType.kPosition);
  }

  public void rollWheel() {
    m_wheel.set(Constants.Intake.INTAKE_ROLL_SPEED);
  }
  
  public void rollBackWheel() {
    m_wheel.set(-Constants.Intake.INTAKE_ROLL_SPEED);
  }
  public void stopWheel() {
    m_wheel.stopMotor();
  } 
  public void resetEncoders(){
    armEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Position", armEncoder.getPosition());
  }

  // ====================COMMANDS====================
  public Command driveCommand (XboxController controller, Intake m_intake){
    return Commands.run(
      ()->this.drive(
        controller.getRawAxis(XboxController.Axis.kRightTrigger.value)
        -controller.getRawAxis(XboxController.Axis.kLeftTrigger.value))
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
    return Commands.runOnce(this::rollWheel, this);
  }
  
  public Command rollBackWheelCommand(){
    return Commands.runOnce(this::rollBackWheel, this);
  }
  public Command stopWheelCommand(){
    return Commands.runOnce(this::stopWheel, this);
  }
}