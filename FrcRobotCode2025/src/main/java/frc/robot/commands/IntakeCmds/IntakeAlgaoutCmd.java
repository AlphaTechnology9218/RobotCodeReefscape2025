package frc.robot.commands.IntakeCmds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.AlgaeIntake;

public class IntakeAlgaOutCmd extends Command{
    
    AlgaeIntake AlgaeIntakeSubsystem;

    public IntakeAlgaOutCmd(AlgaeIntake subsystem){
        this.AlgaeIntakeSubsystem = subsystem;
     
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        AlgaeIntakeSubsystem.intakeDrive(AlgaeIntakeConstants.IntakeRealeaseSpeed);      
        
    }

    @Override
    public void end(boolean interrupted) {
        AlgaeIntakeSubsystem.intakeStop();
    }

    @Override
    public boolean isFinished() {
        if (AlgaeIntakeSubsystem.getCurrent()>30){
        return true;
    }
    else{
        return false;
    }
    }
}