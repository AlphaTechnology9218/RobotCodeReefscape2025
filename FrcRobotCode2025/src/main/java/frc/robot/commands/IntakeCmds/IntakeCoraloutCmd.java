package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class IntakeCoraloutCmd extends Command{
       CoralIntake coralintakeSubsystem;
       Timer timer;
       


    public IntakeCoraloutCmd(CoralIntake subsystem){
        this.coralintakeSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
    }

    @Override
    public void execute() {
        coralintakeSubsystem.intakemotion (-0.4);
        timer.start();
        
    }

    @Override
    public void end(boolean interrupted) {
        coralintakeSubsystem.intakestop();
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > 1.5){
            return true; 
        }
        else{
        return false;
        } 
    }
}