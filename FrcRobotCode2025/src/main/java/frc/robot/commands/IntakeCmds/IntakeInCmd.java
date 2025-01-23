package frc.robot.commands.IntakeCmds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeInCmd extends Command{
    Intake intakeSubsystem;
    double in;
    boolean end;
    public IntakeInCmd(Intake subsystem){
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        in = intakeSubsystem.getIntakeSensorVal();
        intakeSubsystem.IntakeDrive(-1);
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        if(in < 1){
            end = false;
        }else if(in > 1){
            end = true;
        }
        return end;    
    }
}