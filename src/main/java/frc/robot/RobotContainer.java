package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
public class RobotContainer {
  
  public CommandXboxController controller = new CommandXboxController(Constants.IO.ID_CONTROLLER);

  public Chassis m_chassis = new Chassis();
  public Shooter m_shooter = new Shooter();
  public Intake m_intake = new Intake();
  public climber m_climber = new climber();

  public RobotContainer() {
    configureBindings();
    setDefaultCommands();
  }

  private void setDefaultCommands(){
    m_intake.setDefaultCommand(m_intake.setCommand(controller.getLeftY()));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
