package frc.robot.commands.ArmCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;

public class ArmDrivePIDCmd extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm armSubsystem;
  private double spRest, spSource, spL1, spL2, spL3, spL4, sp;
  private final int POV;
  boolean releaseAtSetPoint;

  private PIDController pidController = 
  new PIDController(Arm.getP(),Arm.getI(), Arm.getD());

  public ArmDrivePIDCmd(Arm subsystem, int POV, boolean releaseAtSetPoint){
    this.armSubsystem = subsystem;
    this.spRest = ArmConstants.restSetpoint;
    this.spSource = ArmConstants.sourceSetpoint;
    this.spL1 = ArmConstants.L1setPoint;
    this.spL2 = ArmConstants.L2setPoint;
    this.spL3 = ArmConstants.L3setPoint;
    this.spL4 = ArmConstants.L4setPoint;
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
      case 1:
        sp = spSource;
        break;
      case 2:
        sp = spL1; 
        break;
      case 3:
        sp = spL2;
        break;
      case 4:
        sp = spL3;
        break;
      case 5:
        sp = spL4;
        break;
      }

      pidController.setSetpoint(sp);

      double speed = pidController.calculate(armSubsystem.getEncoder().get());
      if (armSubsystem.getEncoder().get() != 0){
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

  
  
    
}