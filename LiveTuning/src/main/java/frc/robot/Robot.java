// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Robot extends TimedRobot {
  private static final int deviceID = 16;
  private SparkMax m_motor;
  private SparkMaxConfig config;
  private SparkClosedLoopController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, potencia;
  XboxController controle;

  @Override
  public void robotInit() {
    // initialize motor
    m_motor = new SparkMax(deviceID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    controle = new XboxController(0);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    

    /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getClosedLoopController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();


  

    // set PID coefficients  
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(kP)
    .i(kI)
    .d(kD)
    .iZone(kIz)
    .velocityFF(kFF)
    .maxOutput(kMaxOutput)
    .minOutput(kMinOutput)
    .maxMotion.maxAcceleration(750)
    ;
    

    // display PID coefficients on SmartDashboard
    

    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void teleopInit(){
     SmartDashboard.putNumber("P Gain", kP);
     SmartDashboard.putNumber("I Gain", kI);
     SmartDashboard.putNumber("D Gain", kD);
     SmartDashboard.putNumber("I Zone", kIz);
     SmartDashboard.putNumber("Feed Forward", kFF);
     SmartDashboard.putNumber("Max Output", kMaxOutput);
     SmartDashboard.putNumber("Min Output", kMinOutput);
     SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("potencia", potencia);
    SmartDashboard.putNumber("outro", 10);
  }

  @Override
  public void robotPeriodic() {

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);
    double pot = SmartDashboard.getNumber("potencia", 0);
    SmartDashboard.putNumber("pot", pot);

   // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { config.closedLoop.p(p); kP = p; 
   m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);}

   if((pot != potencia)) { potencia = pot;
     m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);}

    if((i != kI)) { config.closedLoop.i(i);kI = i; 
     m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);}

    if((d != kD)) { config.closedLoop.d(d); kD = d; 
     m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);}

    if((iz != kIz)) { config.closedLoop.iZone(iz); kIz = iz; 
     m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);}

    if((ff != kFF)) { config.closedLoop.velocityFF(ff); kFF = ff; 
     m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);}

    if((max != kMaxOutput) || (min != kMinOutput)) { 
      config.closedLoop.maxOutput(max)
      .minOutput(min); 
      kMinOutput = min; kMaxOutput = max; 
      m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    SmartDashboard.putNumber("Setpoint", 1000);
  }

  @Override
  public void teleopPeriodic() {
    
    
    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */

       if(controle.getAButton()){
        m_pidController.setReference( 1000, SparkMax.ControlType.kMAXMotionVelocityControl);
      }else{
        m_pidController.setReference(0, ControlType.kVelocity);  
      }
      SmartDashboard.putBoolean("IsPressed", controle.getAButton());
     
  }

}

