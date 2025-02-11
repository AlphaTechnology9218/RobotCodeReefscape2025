package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;


public class AlgaeIntake extends SubsystemBase{

 SparkMax AlgaeMoveMotor = new SparkMax(AlgaeIntakeConstants.MoveMotorId, MotorType.kBrushless);
 SparkMax AlgaeIntake = new SparkMax(AlgaeIntakeConstants.IntakeMotorId, MotorType.kBrushless);
 SparkMaxConfig Intakeconfig = new SparkMaxConfig();
 SparkMaxConfig Moveconfig = new SparkMaxConfig();
 DutyCycleEncoder dutyEncoder = new DutyCycleEncoder(
    AlgaeIntakeConstants.DutyCycleChannel, 360, 0);
 SparkClosedLoopController pid;
 RelativeEncoder encoder = AlgaeMoveMotor.getEncoder();

public AlgaeIntake(){

    pid = AlgaeMoveMotor.getClosedLoopController();

    Intakeconfig.smartCurrentLimit(20, 40)
    .idleMode(IdleMode.kBrake)
    .openLoopRampRate(0.3)
    .inverted(false);

    Moveconfig.smartCurrentLimit(40,60)
    .idleMode(IdleMode.kBrake)
    .openLoopRampRate(0.3)
    .inverted(false)
    .softLimit.forwardSoftLimitEnabled(false)
    .reverseSoftLimitEnabled(false)
    .forwardSoftLimit(AlgaeIntakeConstants.MoveMotorFwdSoftLimit)
    .reverseSoftLimit(AlgaeIntakeConstants.MoveMotorRvrsSoftLimit);
    

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

public void IntakeMoveDrive(double speed){
    AlgaeMoveMotor.set(speed);
}

public void IntakeMoveStop(){
    AlgaeMoveMotor.stopMotor();
}

public DutyCycleEncoder getEncoder(){
    return dutyEncoder;
}


public double getCurrent(){
    return AlgaeIntake.getOutputCurrent();
}

@Override
public void periodic() {
   SmartDashboard.putNumber
   ("AlgaeIntakePosition", encoder.getPosition());
}
}
