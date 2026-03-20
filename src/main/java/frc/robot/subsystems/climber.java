package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climber extends SubsystemBase {

    private final SparkMax m_climberDown = new SparkMax(Constants.Intake.ID_EXTDENDER, MotorType.kBrushless);
    private final SparkMax m_climberUp = new SparkMax(Constants.Intake.ID_INTAKE_WHEEL, MotorType.kBrushless);   
    
}
