package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.AlgaeIntake;

public class IntakeAlgaINcmd extends Command{
    
   AlgaeIntake AlgaeIntakeSubsystem;


    public IntakeAlgaINcmd(AlgaeIntake subsystem){
        this.AlgaeIntakeSubsystem = subsystem;
        
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
         if(AlgaeIntakeSubsystem.getCurrent() < 30){
            AlgaeIntakeSubsystem.IntakeSetMoveSetPoint(
                AlgaeIntakeConstants.IntakeCollectSetPoint);
         }else{
            AlgaeIntakeSubsystem.IntakeSetMoveSetPoint(
                AlgaeIntakeConstants.intakeHoldSetPoint);
         }
        
    }

    @Override
    public void end(boolean interrupted) {
        AlgaeIntakeSubsystem.intakeStop();
    }

    @Override
    public boolean isFinished() {
    if (AlgaeIntakeSubsystem.getCurrent()>20){
        return true;
    }
    else{
        return false;
    }
      
    }
}
