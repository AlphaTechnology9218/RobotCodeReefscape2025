package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;


public class AlgaeIntake extends SubsystemBase{

 SparkMax AlgaeMoveMotor = new SparkMax(AlgaeIntakeConstants.MoveMotorId, MotorType.kBrushless);
 SparkMax AlgaeIntake = new SparkMax(AlgaeIntakeConstants.IntakeMotorId, MotorType.kBrushless);
 SparkMaxConfig Intakeconfig = new SparkMaxConfig();
 SparkMaxConfig Moveconfig = new SparkMaxConfig();
 SparkClosedLoopController pid;
 DigitalInput limitSwitch = new DigitalInput(AlgaeIntakeConstants.limitSwitchChannel);
 boolean wasResetBylimitSwitch = false;
 RelativeEncoder encoder = AlgaeMoveMotor.getEncoder();

public AlgaeIntake(){

    pid = AlgaeMoveMotor.getClosedLoopController();

    Intakeconfig.smartCurrentLimit(20, 40)
    .idleMode(IdleMode.kBrake)
    .openLoopRampRate(0.3)
    .inverted(false);

    Moveconfig.smartCurrentLimit(20,40)
    .idleMode(IdleMode.kBrake)
    .openLoopRampRate(0.3)
    .inverted(false)
    .softLimit.forwardSoftLimitEnabled(false)
    .reverseSoftLimitEnabled(false)
    .forwardSoftLimit(AlgaeIntakeConstants.MoveMotorFwdSoftLimit)
    .reverseSoftLimit(AlgaeIntakeConstants.MoveMotorRvrsSoftLimit);
    Moveconfig.closedLoop
    .p(0)
    .i(0)
    .d(0)
    .iZone(0);

    AlgaeMoveMotor.configure(Moveconfig, ResetMode.kResetSafeParameters,
     PersistMode.kNoPersistParameters);
    AlgaeIntake.configure(Intakeconfig, ResetMode.kResetSafeParameters,
     PersistMode.kNoPersistParameters);
}
public void intakeDrive(double val){
    AlgaeIntake.set(val);
}
public void intakeStop(){
    AlgaeIntake.stopMotor();

}

public void IntakeSetMoveSetPoint(double setpoint){
    pid.setReference(setpoint, ControlType.kPosition);
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

@Override
public void periodic() {
    resetByLimitSwitch();
   SmartDashboard.putNumber
   ("AlgaeIntakePosition", encoder.getPosition());
}
}
