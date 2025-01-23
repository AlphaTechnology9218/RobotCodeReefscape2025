/*package frc.robot.commands.ArmCmds;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class AmpAnglePIDCmd extends Command{
    ArmSubsystem armSubsystem;

    private PIDController pidController = new PIDController(ArmConstants.kP,ArmConstants.kI, ArmConstants.kD);
    

    public AmpAnglePIDCmd(ArmSubsystem subsystem){
        this.armSubsystem = subsystem;
        pidController.setTolerance(0.1);
        addRequirements(subsystem);

    }

    @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      pidController.setSetpoint(ArmConstants.AmpSetPoint);
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
    return false;
  }
}*/
