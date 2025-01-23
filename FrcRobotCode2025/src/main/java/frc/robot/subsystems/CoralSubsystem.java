package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class CoralSubsystem extends SubsystemBase{

    public enum Setpoint {
        kFeederStation,
        kLevel1,
        kLevel2,
        kLevel3,
        kLevel4;
      }

    SparkMax elevatorLeader = new SparkMax(ArmConstants.arm0ID, MotorType.kBrushless);
    SparkMax elevatorFollower = new SparkMax(ArmConstants.arm1ID, MotorType.kBrushless);
    SparkMaxConfig leaderConfig, followConfig; 
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController armPid;
    double feedfoward;
    boolean wasResetBylimit = false;
    private double elevatorCurrentTarget = ArmConstants.kFeederStation;
    DigitalInput limit = new DigitalInput(3);

    
    public CoralSubsystem(){
        

        leaderConfig = new SparkMaxConfig();
        followConfig = new SparkMaxConfig();

        encoder = elevatorLeader.getEncoder();
        leaderConfig.encoder.positionConversionFactor(ArmConstants.ArmPosConversionFactor);
        leaderConfig.encoder.velocityConversionFactor(ArmConstants.ArmVeloConversionFactor);

        leaderConfig.inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40,60);
        
        leaderConfig.softLimit.forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(ArmConstants.fwdSoftLimit)
        .reverseSoftLimit(ArmConstants.revrsSoftLimit);


        followConfig.follow(elevatorLeader, true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40,60);


        armPid = elevatorLeader.getClosedLoopController();
        leaderConfig.closedLoop
        .p(ArmConstants.Kp)
        .i(ArmConstants.Ki)
        .d(ArmConstants.Kd)
        .iZone(ArmConstants.Kiz)
        .outputRange(ArmConstants.kArmMinOutput,
         ArmConstants.kArmMaxOutput)
        .maxMotion.maxVelocity(ArmConstants.KMaxSpeed)
        .maxAcceleration(ArmConstants.KmaxAcce)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(ArmConstants.MaxAllowedError);
        
         
        
        elevatorLeader.
        configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        elevatorFollower.
        configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        HomeSubsystem();
    }

    public void subuysytemGoToSetpoit(double setpoint){
        armPid.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl,
         null, ArmConstants.ArmFeedFoward);
    }

    private void ResetOnLimitSwitch(){
        if(!wasResetBylimit && limit.get()){
            encoder.setPosition(0);
            wasResetBylimit = true;
        }else if(!limit.get()){
            wasResetBylimit = false;
        }
    }

    private void HomeSubsystem(){
        if(!wasResetBylimit && limit.get()){
            elevatorLeader.stopMotor();
            encoder.setPosition(0);
            wasResetBylimit = true;
        }else if(!limit.get()){
            wasResetBylimit = false;
            elevatorLeader.set(-0.1);
        }
    }

    public Command setSetpointCommand(Setpoint setpoint) {
        return this.runOnce(
            () -> {
              switch (setpoint) {
                case kFeederStation:
                  elevatorCurrentTarget = ArmConstants.kFeederStation;
                  break;
                case kLevel1:

                  elevatorCurrentTarget = ArmConstants.kL1;
                  break;
                case kLevel2:
                  elevatorCurrentTarget = ArmConstants.kL2;
                  break;
                case kLevel3:
                  elevatorCurrentTarget = ArmConstants.kL3;
                  break;
                case kLevel4:
                  elevatorCurrentTarget = ArmConstants.kL4;
                  break;
              }
            });
      }


    public void periodic(){
        ResetOnLimitSwitch();
        double pos = encoder.getPosition();
        double velo = encoder.getVelocity();
        SmartDashboard.putNumber("EncoderCurrentPos", pos);
        SmartDashboard.putNumber("EncoderCurrentVelo", velo);
        SmartDashboard.putNumber("EncoderVeloSetPoint", 30);
    }
        

}
