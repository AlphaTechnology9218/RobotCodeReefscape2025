package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
            AlgaeIntakeSubsystem.IntakeSetMoveSetPoint(1);
         }else{
            AlgaeIntakeSubsystem.IntakeSetMoveSetPoint(2);
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
