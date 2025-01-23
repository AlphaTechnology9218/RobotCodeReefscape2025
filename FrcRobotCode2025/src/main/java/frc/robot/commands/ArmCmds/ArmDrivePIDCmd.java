/*package frc.robot.commands.ArmCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.math.controller.PIDController;

public class ArmDrivePIDCmd extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;
  private double spRest, spSpeaker, spAmp, spPass, sp;
  private final int POV;
  boolean releaseAtSetPoint;

  private PIDController pidController = new PIDController(ArmConstants.kP,ArmConstants.kI, ArmConstants.kD);

  public ArmDrivePIDCmd(ArmSubsystem subsystem, int POV, boolean releaseAtSetPoint){
    this.armSubsystem = subsystem;
    this.spRest = ArmConstants.RestSetPoint;
    this.spSpeaker = ArmConstants.ShooterSetPoint;
    this.spAmp = ArmConstants.AmpSetPoint;
    this.spPass = 0.60;
    this.sp = this.spRest;
    this.POV = POV;
    this.pidController.setTolerance(0.1);
    this.releaseAtSetPoint = releaseAtSetPoint;

    addRequirements(subsystem);
  }
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (POV){
      case 0:
        sp = spRest;
        break;
      case 90:
        sp = spSpeaker;
        break;
      case 180:
        sp = spAmp; 
        break;
      case 270:
        sp = spPass;
        break;
      }
      pidController.setSetpoint(sp);
    //&& armSubsystem.getAbsEncoder().getAbsolutePosition() >= sp
      double speed = pidController.calculate(armSubsystem.getAbsEncoder().getAbsolutePosition() - 
      armSubsystem.getAbsEncoder().getPositionOffset());
      if (armSubsystem.getAbsEncoder().getAbsolutePosition() != 0){
        if (speed < 0 ){
          armSubsystem.armDrive(0.2);
        }
        else{
          armSubsystem.armDrive(-speed);
        }
      }
  
    }
    
  public void end(boolean interrupted) {
    armSubsystem.stopArm();
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint() && releaseAtSetPoint;
  }

  
  
    
}*/