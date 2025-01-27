package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class IntakeCoraloutCmd extends Command{
    
    CoralIntake coralintakeSubsystem;


    public IntakeCoraloutCmd(CoralIntake subsystem){
        this.coralintakeSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
         coralintakeSubsystem.intakemotion (-0.4);
        
    }

    @Override
    public void end(boolean interrupted) {
        coralintakeSubsystem.intakestop();
    }

    @Override
    public boolean isFinished() {
    if (coralintakeSubsystem.getcurrent()>20){
        return true;
    }
    else{
        return false;
    }
      
    }
}
