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

  //public Trigger shooterStopTrigger = new JoystickButton(mech_controller, XboxController.Button.kX.value);
  public Trigger shootTrigger = new JoystickButton(mech_controller, XboxController.Button.kY.value);
  public Trigger shootBackTrigger = new JoystickButton(mech_controller, XboxController.Button.kB.value);
  public Trigger wheelTrigger = new JoystickButton(mech_controller, XboxController.Button.kX.value);
  public Trigger wheelBackTrigger = new JoystickButton(mech_controller, XboxController.Button.kRightBumper.value);
  public Trigger conveyorTrigger = new JoystickButton(mech_controller, XboxController.Button.kA.value);

  public RobotContainer() {
  
    configureBindings();
    defaultCommands();
  
}

  private void configureBindings() {
    stopChassisTrigger.onTrue(new SequentialCommandGroup(
      m_chassis.stopCommand(), 
      m_chassis.clearFaultsCommand()
    ));

    conveyorTrigger.onTrue(m_shooter.conveyorCommand()); //!!
    conveyorTrigger.onFalse(m_shooter.stopShooterCommand());

    // shootTrigger.onTrue(m_shooter.driveShooterCommand(750));
    shootTrigger.onTrue(m_shooter.dumbTestFrontCommand());
    shootTrigger.onFalse(m_shooter.stopShooterCommand());
    shootBackTrigger.onTrue(m_shooter.dumbTestBackCommand());
    shootBackTrigger.onFalse(m_shooter.stopShooterCommand());
    
    wheelTrigger.onTrue(m_intake.rollWheelCommand());
    wheelTrigger.onFalse(m_intake.stopWheelCommand());
    wheelBackTrigger.onTrue(m_intake.rollBackWheelCommand());
    wheelBackTrigger.onFalse(m_intake.stopWheelCommand());
  }
  
  
  private void defaultCommands() {
    m_chassis.setDefaultCommand(
        m_chassis.driveCommand(drive_controller, m_chassis)
    );

    m_shooter.setDefaultCommand(
      m_shooter.stopShooterCommand()
    );

    m_intake.setDefaultCommand(
      m_intake.driveCommand(mech_controller, m_intake)
    );
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
