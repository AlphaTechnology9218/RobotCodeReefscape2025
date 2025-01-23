package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeIntake extends SubsystemBase{
 SparkMax AlgaeIntake = new SparkMax(5, MotorType.kBrushless);
 SparkMaxConfig config = new SparkMaxConfig();
 DigitalInput limitSwitch = new DigitalInput(6);
 boolean wasResetBylimitSwitch = false;
 RelativeEncoder encoder = AlgaeIntake.getEncoder();

public AlgaeIntake(){
    config.smartCurrentLimit(20, 40)
    .idleMode(IdleMode.kBrake)
    .openLoopRampRate(.10)
    .softLimit.forwardSoftLimitEnabled(true)
    .reverseSoftLimitEnabled(true)
    .forwardSoftLimit(50)
    .reverseSoftLimit(0);

}
public void intakeDrive(double val){
    AlgaeIntake.set(val);
}
public void intakeStop(){
    AlgaeIntake.stopMotor();

}

public void resetByLimitSwitch(){
    if (!wasResetBylimitSwitch && limitSwitch.get()) {
       encoder.setPosition(0);
       wasResetBylimitSwitch = true;  
    }else if(!limitSwitch.get()){
        wasResetBylimitSwitch = false;
    }
}
public double getCurrent(){
    return AlgaeIntake.getOutputCurrent();
}

public void periodic(){
   resetByLimitSwitch();
   SmartDashboard.putNumber
   ("AlgaeIntakePosition", encoder.getPosition());
}
}
