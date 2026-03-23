package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
public class RobotContainer {
  
  public Chassis m_chassis = new Chassis();
  public Shooter m_shooter = new Shooter();
  public Intake m_intake = new Intake();
  public climber m_climber = new climber();
  public XboxController drive_controller = new XboxController(Constants.IO.ID_DRIVER_CHASSIS);
  public XboxController mech_controller = new XboxController(Constants.IO.ID_DRIVER_MECH);  
  private final SendableChooser<Command> autoChooser;
  
  // Chassis Triggers
  public Trigger stopChassisTrigger = new JoystickButton(drive_controller, XboxController.Button.kX.value);

  // Mech Triggers
  public Trigger wheelTrigger = new JoystickButton(mech_controller, XboxController.Button.kLeftBumper.value);
  public Trigger wheelBackTrigger = new JoystickButton(mech_controller, XboxController.Button.kRightBumper.value);
  public Trigger conveyorTrigger = new JoystickButton(mech_controller, XboxController.Button.kA.value);
  public Trigger shootPIDTriggerMid = new JoystickButton(mech_controller, XboxController.Button.kY.value);
  public Trigger shootPIDTriggerClose = new JoystickButton(mech_controller, XboxController.Button.kX.value);
  public Trigger shootPIDTriggerFar = new JoystickButton(mech_controller, XboxController.Button.kB.value);
  public Trigger extendIntakeTrigger = new Trigger(()->mech_controller.getLeftY()<-0.6);
  public Trigger retractIntakeTrigger = new Trigger(()->mech_controller.getLeftY()>0.6);
  public Trigger climberUpTrigger = new Trigger(()->drive_controller.getRightY()>0.6);
  public Trigger climberDownTrigger = new Trigger(()->drive_controller.getRightY()<-0.6);


  public RobotContainer() {
    boolean isCompetition = DriverStation.isFMSAttached();
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream)-> isCompetition
        ? stream.filter(auto->auto.getName().contains("Comp"))
        : stream
    );
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
    defaultCommands();
  }

  private void configureBindings() {
    stopChassisTrigger.onTrue(new SequentialCommandGroup(
      m_chassis.stopCommand(), 
      m_chassis.clearFaultsCommand()
    ));

    conveyorTrigger.onTrue(m_shooter.conveyorCommand());
    conveyorTrigger.onFalse(m_shooter.stopShooterCommand());

    shootPIDTriggerClose.onTrue(m_shooter.shooterPIDCommandClose());
    shootPIDTriggerClose.onFalse(m_shooter.stopShooterCommand());

    shootPIDTriggerMid.onTrue(m_shooter.shooterPIDCommandMid());
    shootPIDTriggerMid.onFalse(m_shooter.stopShooterCommand());

    shootPIDTriggerFar.onTrue(m_shooter.shooterPIDCommandFar());
    shootPIDTriggerFar.onFalse(m_shooter.stopShooterCommand());

    wheelTrigger.onTrue(m_intake.rollWheelCommand());
    wheelTrigger.onFalse(m_intake.stopWheelCommand());

    wheelBackTrigger.onTrue(m_intake.rollBackWheelCommand());
    wheelBackTrigger.onFalse(m_intake.stopWheelCommand());

    extendIntakeTrigger.onTrue(m_intake.extendCommand());
    retractIntakeTrigger.onTrue(m_intake.retractCommand());

    climberDownTrigger.whileTrue(m_climber.dumbReverseClimber());
    climberUpTrigger.whileTrue(m_climber.dumbClimber());
  
    Command resetArmEncoderCommand = m_intake.resetEncoderCommand();
    SmartDashboard.putData("Reset Arm Encoder", resetArmEncoderCommand);
  }
  
  private void defaultCommands() {
    m_chassis.setDefaultCommand(
        m_chassis.driveCommand(drive_controller, m_chassis)
    );
    m_climber.setDefaultCommand(m_climber.stopCommandClimber());
    m_intake.setDefaultCommand(m_intake.driveCommand(mech_controller));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
