package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.*;
public class RobotContainer {
  // subsystems
  public Chassis m_chassis = new Chassis();
  public Shooter m_shooter = new Shooter();
  public Intake m_intake = new Intake();
  public climber m_climber = new climber();
  
  public XboxController drive_controller = new XboxController(Constants.IO.ID_DRIVER_CHASSIS);
  public XboxController mech_controller = new XboxController(Constants.IO.ID_DRIVER_MECH);  
  public XboxController test_controller = new XboxController(Constants.IO.ID_TEST_CONTROLLER);
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

  // Test Triggers
  public Trigger testTriggerQuasistatic = new JoystickButton(test_controller, XboxController.Button.kY.value);
  public Trigger testTriggerQuasistaticReverse = new JoystickButton(test_controller, XboxController.Button.kX.value);
  public Trigger testTriggerDynamic = new JoystickButton(test_controller, XboxController.Button.kA.value);
  public Trigger testTriggerDynamicReverse = new JoystickButton(test_controller, XboxController.Button.kB.value);

  public RobotContainer() {
    boolean isCompetition = DriverStation.isFMSAttached();
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream)-> isCompetition
        ? stream.filter(auto->auto.getName().contains("Comp"))
        : stream
    );
    SmartDashboard.putData("Auto Chooser", autoChooser);
    registerCommands();
    SmartDashboard.putBoolean("Is Competition Mode", isCompetition);
    configureBindings();
    defaultCommands();
  }

  private void registerCommands() {
    NamedCommands.registerCommand("shootFar", m_shooter.shooterCommandFar());
    NamedCommands.registerCommand("shootMid", m_shooter.shooterCommandMid());
    NamedCommands.registerCommand("shootClose", m_shooter.shooterCommandClose());
    NamedCommands.registerCommand("stopShooter", m_shooter.stopShooterCommand());
    NamedCommands.registerCommand("conveyorMove", m_shooter.conveyorCommand());
    NamedCommands.registerCommand("stopConveyor", m_shooter.stopConveyorCommand());
    NamedCommands.registerCommand("readyIntake", m_intake.readyIntake());
  }

  private void configureBindings() {
    stopChassisTrigger.onTrue(new SequentialCommandGroup(
      m_chassis.stopCommand(), 
      m_chassis.clearFaultsCommand()
    ));

    conveyorTrigger.onTrue(m_shooter.conveyorCommand());
    conveyorTrigger.onFalse(m_shooter.stopConveyorCommand());

    /*Shooter */

    shootPIDTriggerClose.onTrue(m_shooter.shooterCommandClose());
    shootPIDTriggerClose.onFalse(m_shooter.stopShooterCommand());

    shootPIDTriggerMid.onTrue(m_shooter.shooterCommandMid());
    shootPIDTriggerMid.onFalse(m_shooter.stopShooterCommand());

    shootPIDTriggerFar.onTrue(m_shooter.shooterCommandFar());
    shootPIDTriggerFar.onFalse(m_shooter.stopShooterCommand());


    /* Intake */

    wheelTrigger.onTrue(m_intake.rollWheelCommand());
    wheelTrigger.onFalse(m_intake.stopWheelCommand());

    wheelBackTrigger.onTrue(m_intake.rollBackWheelCommand());
    wheelBackTrigger.onFalse(m_intake.stopWheelCommand());

    extendIntakeTrigger.onTrue(m_intake.extendCommand());
    retractIntakeTrigger.onTrue(m_intake.retractCommand());

    Command resetArmEncoderCommand = m_intake.resetEncoderCommand();
    SmartDashboard.putData("Reset Arm Encoder", resetArmEncoderCommand);

    /* Climber */
    climberDownTrigger.whileTrue(m_climber.dumbReverseClimberCommand());
    climberUpTrigger.whileTrue(m_climber.dumbClimberCommand());

    /* Test Routines */
    testTriggerDynamic.whileTrue(m_chassis.sysIdDynamic(SysIdRoutine.Direction.kForward));
    testTriggerDynamicReverse.whileTrue(m_chassis.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    testTriggerQuasistatic.whileTrue(m_chassis.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    testTriggerQuasistaticReverse.whileTrue(m_chassis.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

  }
  
  private void defaultCommands() {
    m_chassis.setDefaultCommand(
        m_chassis.driveCommand(drive_controller, m_chassis)
    );
    m_climber.setDefaultCommand(m_climber.stopCommandClimberCommand());
    m_intake.setDefaultCommand(m_intake.driveCommand(mech_controller));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
