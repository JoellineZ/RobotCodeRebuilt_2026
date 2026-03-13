package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.*;
public class RobotContainer {
    
  public Chassis m_chassis = new Chassis();
  public Shooter m_shooter = new Shooter();
  public Intake m_intake = new Intake();
  public climber m_climber = new climber();
  public XboxController drive_controller = new XboxController(Constants.ID_DRIVER_CHASSIS);
  public XboxController mech_controller = new XboxController(Constants.ID_DRIVER_MECH);


  
  public Trigger stopChassisTrigger = new JoystickButton(drive_controller, XboxController.Button.kX.value);

  public Trigger lineFuelTrigger = new JoystickButton(mech_controller, XboxController.Button.kA.value);
  //public Trigger shooterStopTrigger = new JoystickButton(mech_controller, XboxController.Button.kX.value);
  public Trigger shootTrigger = new JoystickButton(mech_controller, XboxController.Button.kY.value);

public RobotContainer() {
  
    configureBindings();
    defaultCommands();
  
}

  private void configureBindings() {
    stopChassisTrigger.onTrue(new SequentialCommandGroup(
      m_chassis.stopCommand(), 
      m_chassis.clearFaultsCommand()
    ));
    lineFuelTrigger.onTrue(m_shooter.lineFuelCommand());
    shootTrigger.onTrue(m_shooter.MainshooterCommand());
    //shooterStopTrigger.onTrue(m_shooter.stopShooterCommand());
  }
  
  private void defaultCommands() {
    m_chassis.setDefaultCommand(
        m_chassis.driveCommand(drive_controller)
    );
    m_shooter.setDefaultCommand(m_shooter.stopShooterCommand());
  }
}
