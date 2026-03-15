package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
public class RobotContainer {
  
  public CommandXboxController controller = new CommandXboxController(Constants.IO.ID_CONTROLLER);

  public Chassis m_chassis = new Chassis();
  public Shooter m_shooter = new Shooter();
  public Intake m_intake = new Intake();
  public climber m_climber = new climber();
  public XboxController drive_controller = new XboxController(Constants.IO.ID_DRIVER_CHASSIS);
  public XboxController mech_controller = new XboxController(Constants.IO.ID_DRIVER_MECH);


  
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
    shootTrigger.whileTrue(m_shooter.MainshooterCommand(Constants.Shooter.MAX_RPM));
  }
  
  private void defaultCommands() {
    m_chassis.setDefaultCommand(
        m_chassis.driveCommand(drive_controller)
    );
    m_shooter.setDefaultCommand(m_shooter.stopShooterCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
