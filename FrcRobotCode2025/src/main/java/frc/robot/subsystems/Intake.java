package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LiveTuning;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase{
        
        AnalogPotentiometer intakeInfraRedSensor = new AnalogPotentiometer(IntakeConstants.SensorAnalogChannel, 5, -0.4);
        SparkMax IntakeMotor = new SparkMax(IntakeConstants.IntakeMotorID, MotorType.kBrushless);
        SparkClosedLoopController pid = IntakeMotor.getClosedLoopController();
        SparkMaxConfig motorConfig;
        double kp,ki,kd;

        LiveTuning tune = new LiveTuning(kp, ki, kd, "Intake");

        public Intake(){
            IntakeStop();

            motorConfig = new SparkMaxConfig();

            motorConfig.inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40, 60)
            .closedLoop.p(tune.Kp)
            .i(tune.Ki)
            .d(tune.Kd);

            IntakeMotor.configure
            (motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

            
        }
        
        public void IntakeDrive(double val){
            IntakeMotor.set(val);
            
        }

      
        public void IntakeStop(){
            IntakeMotor.set(0);
            
        }

        public double getCurrent(){
            return IntakeMotor.getOutputCurrent();
        }

        
        public double getIntakeSensorVal(){
            double val = intakeInfraRedSensor.get();
            return val; 
        }

        @Override
        public void periodic() {
            SmartDashboard.putNumber("Numerical Sensor Value Test", getIntakeSensorVal());
            tune.updatePID();
            SmartDashboard.putNumber("Changed P", tune.Kp);
            SmartDashboard.putNumber("Changed I", tune.Ki);
            SmartDashboard.putNumber("Changed D", tune.Kd);
            
        }

        
}