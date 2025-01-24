package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LiveTuning;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    SparkMax armMotor = new SparkMax(ArmConstants.ArmMotorID, MotorType.kBrushless); 
    SparkMaxConfig armConfig; 
    static double p, i, d;
    DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.EncoderDigitalSource, 360, 0);
    LiveTuning tuning = new LiveTuning(p, i, d, "Arm");

    Arm(){
        armConfig = new SparkMaxConfig();

        armConfig.smartCurrentLimit(40, 60)
        .openLoopRampRate(0.3)
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .softLimit.forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(ArmConstants.fwdSoftLimit)
        .reverseSoftLimit(ArmConstants.rvrsSoftLimit);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void armDrive(double potency){
        armMotor.set(potency);
    }

    public void stopArm(){
        armMotor.stopMotor();
    }

    public DutyCycleEncoder getEncoder(){
        return encoder;
    }

    public static double getP(){
        return p;
    }

    public static double getI(){
        return i;
    }

    public static double getD(){
        return d;
    }
    @Override
    public void periodic() {
        tuning.updatePID();
    }
    
}
