package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
public class Intake extends SubsystemBase {
  private final SparkMax m_arm = new SparkMax(Constants.ID_INTAKE_ARM, MotorType.kBrushless);
  private final SparkMax m_wheel = new SparkMax(Constants.ID_INTAKE_WHEEL, MotorType.kBrushless);   
  private SparkMaxConfig armConfig = new SparkMaxConfig();
  private SparkMaxConfig wheelConfig = new SparkMaxConfig();
  
  private final RelativeEncoder armEncoder= m_arm.getEncoder();
  
  public Intake() {
      armConfig.idleMode(IdleMode.kBrake);
      wheelConfig.idleMode(IdleMode.kBrake);
      armConfig.inverted(false);
      wheelConfig.inverted(true);
      m_arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_wheel.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      //resetencoders()
  }

  
  public void rollWheel() {
    m_wheel.set(Constants.INTAKE_WHEEL_SPEED);
  }
  public void drive(double drive_speed){
    if(Math.abs(drive_speed) > 1 ){
      drive_speed = drive_speed > 0 ? 1 : -1;
    }
    double motor_speed = drive_speed * Constants.MAX_ARM_SPEED_PERCENT;
    motor_speed =  stopAtLimit(motor_speed);

    m_arm.set(motor_speed);
  }

  public void resetEncoders(){
    armEncoder.setPosition(0);
      }

  public Command driveCommand (XboxController controller){
    return Commands.run(
      ()->this.drive(
        controller.getRawAxis(XboxController.Axis.kRightTrigger.value)
        -controller.getRawAxis(XboxController.Axis.kLeftTrigger.value))
      , this);
  }
   private double stopAtLimit(double input){
    double output = input;
    if (this.armEncoder.getPosition() <= 0 && input < 0){
      output = 0; 
    }
    return output;
  }
}
