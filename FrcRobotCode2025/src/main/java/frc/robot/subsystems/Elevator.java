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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LiveTuning;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{

    public enum Setpoint {
        kFeederStation,
        kLevel1,
        kLevel2,
        kLevel3,
        kLevel4;
      }

    SparkMax elevatorLeader = new SparkMax(ElevatorConstants.Elevator0ID, MotorType.kBrushless);
    SparkMax elevatorFollower = new SparkMax(ElevatorConstants.Elevator1ID, MotorType.kBrushless);
    ElevatorFeedforward ElevFeedFoward = new ElevatorFeedforward(0, 0, 0);
    SparkMaxConfig leaderConfig, followConfig;
    double kp,ki,kd; 
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController ElevatorPid;
    double feedfoward;
    boolean wasResetBylimit = false;
    private double elevatorCurrentTarget = ElevatorConstants.kFeederStation;
    DigitalInput limit = new DigitalInput(3);
    LiveTuning tuning;

    
    public Elevator(){


        tuning = new LiveTuning(kp, ki, kd, "Elevator");

        leaderConfig = new SparkMaxConfig();
        followConfig = new SparkMaxConfig();

        encoder = elevatorLeader.getEncoder();
        leaderConfig.encoder.positionConversionFactor(ElevatorConstants.ArmPosConversionFactor);
        leaderConfig.encoder.velocityConversionFactor(ElevatorConstants.ArmVeloConversionFactor);

        leaderConfig.inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40,60)
        .softLimit.forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(ElevatorConstants.fwdSoftLimit)
        .reverseSoftLimit(ElevatorConstants.revrsSoftLimit);


        followConfig.follow(elevatorLeader, true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40,60);


        ElevatorPid = elevatorLeader.getClosedLoopController();
        leaderConfig.closedLoop
        .p(kp)
        .i(ki)
        .d(kd)
        .iZone(ElevatorConstants.Kiz)
        .outputRange(ElevatorConstants.kArmMinOutput,
         ElevatorConstants.kArmMaxOutput)
        .maxMotion.maxVelocity(ElevatorConstants.KMaxSpeed)
        .maxAcceleration(ElevatorConstants.KmaxAcce)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(ElevatorConstants.MaxAllowedError);
        
         
        
        elevatorLeader.
        configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        elevatorFollower.
        configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        HomeSubsystem();
    }

    public void subuysytemGoToSetpoit(double setpoint){
        feedfoward = ElevFeedFoward.calculate(ElevatorConstants.KMaxSpeed, ElevatorConstants.KmaxAcce);
        ElevatorPid.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl,
         null, feedfoward);
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
                  elevatorCurrentTarget = ElevatorConstants.kFeederStation;
                  break;
                case kLevel1:

                  elevatorCurrentTarget = ElevatorConstants.kL1;
                  break;
                case kLevel2:
                  elevatorCurrentTarget = ElevatorConstants.kL2;
                  break;
                case kLevel3:
                  elevatorCurrentTarget = ElevatorConstants.kL3;
                  break;
                case kLevel4:
                  elevatorCurrentTarget = ElevatorConstants.kL4;
                  break;
              }
            });
      }


    @Override
    public void periodic() {
      ResetOnLimitSwitch();
      tuning.updatePID();
      double pos = encoder.getPosition();
      double velo = encoder.getVelocity();
      SmartDashboard.putNumber("EncoderCurrentPos", pos);
      SmartDashboard.putNumber("EncoderCurrentVelo", velo);
      SmartDashboard.putNumber("EncoderVeloSetPoint", 30);
    }
        

}
