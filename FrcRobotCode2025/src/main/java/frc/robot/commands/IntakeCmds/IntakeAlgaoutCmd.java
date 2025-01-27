package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class IntakeAlgaoutCmd extends Command{
    
    AlgaeIntake algaintakeSubsystem;


    public IntakeAlgaoutCmd(AlgaeIntake subsystem){
        this.algaintakeSubsystem = subsystem;
        
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
         algaintakeSubsystem.intakeDrive (-0.4);
        
    }

    @Override
    public void end(boolean interrupted) {
        algaintakeSubsystem.intakeStop();
    }

    @Override
    public boolean isFinished() {
    if (algaintakeSubsystem.getCurrent()>20){
        return true;
    }
    else{
        return false;
    }
      
    }
}
