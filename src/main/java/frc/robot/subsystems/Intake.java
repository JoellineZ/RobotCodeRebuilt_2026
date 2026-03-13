package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    SparkMax extender = new SparkMax(Constants.Intake.ID_EXTD_NEO, MotorType.kBrushless);
    RelativeEncoder encoder = extender.getEncoder();
    double position=0;
    SparkMaxConfig brakeConfig = new SparkMaxConfig();
    SparkMaxConfig coastConfig = new SparkMaxConfig();

    public Intake(){
        encoder.setPosition(0); // Posicion Predeterminada antes de encender
        brakeConfig.idleMode(IdleMode.kBrake);
        coastConfig.idleMode(IdleMode.kCoast);
        if(RobotState.isTest()){
            setCoast();
        }else{
            setBrake();
        }
    }

    double getEncoder(){
        return encoder.getPosition();
    }

    void set(double speed){ // speed = [-1,1]
        double v = Math.max(-1, Math.min(1, speed));
        if(position>=Constants.Intake.LIMIT_FWD || position<=Constants.Intake.LIMIT_BWD) v = 0;
        extender.set(v);
    }

    void setBrake(){
        extender.configure(brakeConfig, ResetMode.kResetSafeParameters, null);
    }

    void setCoast(){
        extender.configure(coastConfig, ResetMode.kResetSafeParameters, null);
    }

    @Override
    public void periodic() {
        position = getEncoder();
        SmartDashboard.putNumber("Extender Position", position);
    }

    public Command setCommand(double speed){
        return run(()->set(speed));
    }

}
