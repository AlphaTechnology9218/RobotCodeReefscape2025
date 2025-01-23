/*package frc.robot.commands.ArmCmds;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ShooterAnglePIDCmd extends Command{
    ArmSubsystem armSubsystem;

    private PIDController pidController = new PIDController(ArmConstants.kP,ArmConstants.kI, ArmConstants.kD);
    

    public ShooterAnglePIDCmd(ArmSubsystem subsystem){
        this.armSubsystem = subsystem;
        
        addRequirements(subsystem);

    }

    @Override
  public void initialize() {
    pidController.reset();
    pidController.setTolerance(0.1);
      pidController.setSetpoint(ArmConstants.ShooterSetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
