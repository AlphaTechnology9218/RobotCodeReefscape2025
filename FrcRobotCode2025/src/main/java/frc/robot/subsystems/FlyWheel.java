package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;


public class FlyWheel extends SubsystemBase{

    SparkMax flyWheelLeader = new SparkMax(FlywheelConstants.flyWheelMotorID0, MotorType.kBrushless);
    SparkMax flyWheelFollower = new SparkMax(FlywheelConstants.flyWheelMotorID1, MotorType.kBrushless);
    SparkMaxConfig motorLeaderConfig;
    SparkMaxConfig motorFollowConfig;
    private final SparkClosedLoopController pid;
    private final RelativeEncoder encoder;
    private  double setpoint;

    public FlyWheel(){
        FlyWheelStop();

        motorLeaderConfig = new SparkMaxConfig();
        motorFollowConfig = new SparkMaxConfig();

        motorLeaderConfig.inverted(false)
        .smartCurrentLimit(40, 60)
        .idleMode(IdleMode.kCoast);

        motorFollowConfig.follow(flyWheelLeader, true)
        .smartCurrentLimit(40, 60)
        .idleMode(IdleMode.kCoast);

        
        motorLeaderConfig.encoder
        .positionConversionFactor(FlywheelConstants.flyWheelConversionFactor)
        .velocityConversionFactor(FlywheelConstants.flyWheelConversionFactor);
        encoder = flyWheelLeader.getEncoder();
        
        
        motorLeaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(FlywheelConstants.Kp, FlywheelConstants.Ki, FlywheelConstants.Kd)
        .velocityFF(1/473);
        pid = flyWheelLeader.getClosedLoopController();

        flyWheelLeader.configure
        (motorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        flyWheelFollower.configure
        (motorFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
       
         
        
    } 

    public void FlyWheelActive(double setpointRPM){
        pid.setReference(setpointRPM
        , ControlType.kVelocity);
        setpoint = setpointRPM;
    }

    

    public void FlyWheelStop(){
        flyWheelLeader.stopMotor();
    }

    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FlyWheelMotorVelocity", encoder.getVelocity());
        SmartDashboard.putNumber("FlyWheelSetPoint", setpoint);   
    }

    }


