package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntake extends SubsystemBase{
    SparkMax intakecoral = new SparkMax (CoralIntakeConstants.IntakeMotorId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig ();

        public CoralIntake(){
             config.smartCurrentLimit(20, 40)
             .idleMode(IdleMode.kBrake)
             .openLoopRampRate(.10)
             .inverted(false);

             intakecoral.configure(config, ResetMode.kResetSafeParameters,
              PersistMode.kNoPersistParameters);
             
        }
        public void intakemotion ( double vel){
            intakecoral.set (vel);
        }
        public void intakestop (){
            intakecoral.stopMotor();
        }
        
        
        public double getcurrent (){
            return intakecoral.getOutputCurrent();
            
        }

        @Override
        public void periodic() {
            SmartDashboard.putNumber("CoralIntakeCurrent", getcurrent());
        }

}
