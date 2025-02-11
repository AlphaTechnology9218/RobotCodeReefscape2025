package frc.robot.commands.ArmCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;

public class ArmDrivePIDCmd extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm armSubsystem;
  private double spRest, spSource, spWCollect, spWScore, spL1,
   spL2, spL3, spL4, spS, spW;
  private final int POV;
  boolean releaseAtSetPoint;

  private PIDController shoulderPidController = 
  new PIDController(Arm.getsP(),Arm.getsI(), Arm.getsD());
  private PIDController wristPidController =
   new PIDController(Arm.getwP(), Arm.getwI(), Arm.getwD());
   
  public ArmDrivePIDCmd(Arm subsystem, int POV, boolean releaseAtSetPoint){
    this.armSubsystem = subsystem;
    this.spRest = ArmConstants.restSetpoint;
    this.spSource = ArmConstants.sourceSetpoint;
    this.spWCollect = ArmConstants.WristCollect;
    this.spWScore = ArmConstants.WristScore;
    this.spL1 = ArmConstants.L1setPoint;
    this.spL2 = ArmConstants.L2setPoint;
    this.spL3 = ArmConstants.L3setPoint;
    this.spL4 = ArmConstants.L4setPoint;
    this.spS = this.spRest;
    this.spW = this.spWScore;
    this.POV = POV;
    this.shoulderPidController.setTolerance(0.1);
    this.releaseAtSetPoint = releaseAtSetPoint;

    addRequirements(subsystem);
  }
  @Override
  public void initialize() {
    shoulderPidController.reset();
    wristPidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (POV){
      case 0:
        spS = spRest;
        spW = spWScore;
        break;
      case 1:
        spS = spSource;
        spW = spWCollect;
        break;
      case 2:
        spS = spL1;
        spW = spWScore; 
        break;
      case 3:
        spS = spL2;
        spW = spWScore;
        break;
      case 4:
        spS = spL3;
        spW = spWScore;
        break;
      case 5:
        spS = spL4;
        spW = spWScore;
        break;
      }

      shoulderPidController.setSetpoint(spS);
      wristPidController.setSetpoint(spW);

      double speedS = shoulderPidController.calculate(armSubsystem.getShoulderEncoder().get());
      double speedW = wristPidController.calculate(armSubsystem.getWristEncoder().get());
      
      if (armSubsystem.getShoulderEncoder().get() != 0){
        if (speedS < 0 ){
          armSubsystem.ShouderDrive(0.2);
          armSubsystem.WristDrive(speedW);
          }
        else{
          armSubsystem.ShouderDrive(speedS);
          armSubsystem.WristDrive(speedW);
          }
      }
  
      }
    
  public void end(boolean interrupted) {
    armSubsystem.stopArm();;
  }

  @Override
  public boolean isFinished() {
    return shoulderPidController.atSetpoint() && releaseAtSetPoint;
  }

  
  
    
}