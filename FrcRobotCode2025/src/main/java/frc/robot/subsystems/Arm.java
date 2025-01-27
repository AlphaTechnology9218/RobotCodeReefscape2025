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

    SparkMax ShoulderMotor = new SparkMax(ArmConstants.ShoulderMotorID, MotorType.kBrushless);
    SparkMax WristMotor = new SparkMax(ArmConstants.WristMotorID, MotorType.kBrushless); 
    SparkMaxConfig ShoulderConfig;
    SparkMaxConfig WristConfig; 
    static double Sp, Si, Sd, Wp, Wi, Wd;
    DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(ArmConstants.ShoulderDigitalSource, 360, 0);
    DutyCycleEncoder wristEncoder = new DutyCycleEncoder(ArmConstants.WristDigitalSource,360, 0);
    LiveTuning tuningSholder = new LiveTuning(Sp, Si, Sd, "Shoulder");
    LiveTuning tuningWrist = new LiveTuning(Wp, Wi, Wd, "Wrist");

    public Arm(){
        ShoulderConfig = new SparkMaxConfig();
        WristConfig = new SparkMaxConfig();

        ShoulderConfig.smartCurrentLimit(40, 60)
        .openLoopRampRate(0.3)
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .encoder.
        positionConversionFactor(0)
        .velocityConversionFactor(0);

        ShoulderConfig.softLimit.forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(ArmConstants.SfwdSoftLimit)
        .reverseSoftLimit(ArmConstants.SrvrsSoftLimit);

        WristConfig.smartCurrentLimit(40, 60)
        .openLoopRampRate(0.3)
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .encoder.
        positionConversionFactor(0)
        .velocityConversionFactor(0);

        WristConfig.softLimit.forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(ArmConstants.WfwdSoftLimit)
        .reverseSoftLimit(ArmConstants.WrvrsSoftLimit);

        ShoulderMotor.configure(ShoulderConfig,
         ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        WristMotor.configure(WristConfig,
         ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void ShouderDrive(double speed){
        ShoulderMotor.set(speed);
    }

    public void WristDrive(double speed){
        WristMotor.set(speed);
    }

    public void stopArm(){
        ShoulderMotor.stopMotor();
        WristMotor.stopMotor();
    }


    public DutyCycleEncoder getShoulderEncoder(){
        return shoulderEncoder;
    }

    public DutyCycleEncoder getWristEncoder(){
        return wristEncoder;
    }

    public static double getsP(){
        return Sp;
    }

    public static double getsI(){
        return Si;
    }

    public static double getsD(){
        return Sd;
    }

    public static double getwP(){
        return Wp;
    }

    public static double getwI(){
        return Wi;
    }

    public static double getwD(){
        return Wd;
    }

    @Override
    public void periodic() {
        tuningSholder.updatePID();
        tuningWrist.updatePID();
    }
    
}
