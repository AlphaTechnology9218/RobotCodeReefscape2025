// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSystemControllerPort = 1;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class SwerveConstants{
    public static final double kSwerveSteeringRatio = 21.428571428571428571428571428571;
    public static final double ROBOT_MASS = 40.5; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, 0), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(14.5);
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(6.5,0.00000008,1.39);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0, 0, 0);
    public static RobotConfig robotConfig;

      // Maximum speed of the robot in meters per second, used to limit acceleration.
  }

  public static final class AutonConstants
  {
    
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class ElevatorConstants
  
  {
    
    public static final int Elevator0ID = 14;
    public static final int Elevator1ID  = 15;
    public static final double KMaxSpeed = 1;
    public static final double KmaxAcce = 0;
    public static final double Kp = 1;
    public static final double Ki = 0;
    public static final double Kd = 0.1;
    public static final double Kiz = 0;
    public static final double kArmMaxOutput = 1;
    public static final double kArmMinOutput = -1;
    public static final double ArmPosConversionFactor = 0;
    public static final double ArmVeloConversionFactor = 0;
    public static final double fwdSoftLimit = 0;
    public static final double revrsSoftLimit = 0;
    public static final double kHomePosition = 0.0;
    public static final double MaxAllowedError= 0.4;
    public static final double ElevatorFeedFoward = 0.0;
    public static final double kFeederStation = 0;
    public static final double kL1 = 0;
    public static final double kL2 = 0;
    public static final double kL3 = 0;
    public static final double kL4 = 0;


  }

  public static final class CoralIntakeConstants
  {
      public static final int IntakeMotorId = 16;
  }

  public static final class AlgaeIntakeConstants
  {
      public static final int IntakeMotorId = 18;
      public static final int MoveMotorId = 17;
      public static final int limitSwitchChannel = 6;
      public static final double MoveMotorFwdSoftLimit = 50;
      public static final double MoveMotorRvrsSoftLimit = 50;
  }

  public static final class ArmConstants
  {
    public static final int ArmMotorID = 19;
    public static final int EncoderDigitalSource = 5;
    public static final double fwdSoftLimit = 0;
    public static final double rvrsSoftLimit = 0;
    public static final double restSetpoint = 0;
    public static final double sourceSetpoint = 0;
    public static final double L1setPoint = 0;
    public static final double L2setPoint = 0;
    public static final double L3setPoint = 0;
    public static final double L4setPoint = 0;
    public static final double Kp = 0;
    public static final double Ki = 0;
    public static final double Kd = 0;
  }
}
