package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class IntakeAlgaINcmd extends Command{
    
   AlgaeIntake AlgaeIntakeSubsystem;


    public IntakeAlgaINcmd(){
      
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
         AlgaeIntakeSubsystem.intakeDrive (0.4);
        
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
