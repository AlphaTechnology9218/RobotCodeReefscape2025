package frc.robot.commands.IntakeCmds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.AlgaeIntake;

public class IntakeAlgaINcmd extends Command{
    
    AlgaeIntake AlgaeIntakeSubsystem;
    private double sp, spCollect;
    private PIDController intakePidController = new PIDController(AlgaeIntakeConstants.kP,
    AlgaeIntakeConstants.kI, AlgaeIntakeConstants.kD);

    public IntakeAlgaINcmd(AlgaeIntake subsystem){
        this.AlgaeIntakeSubsystem = subsystem;
        this.spCollect = AlgaeIntakeConstants.spCollect;
        this.sp = spCollect;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intakePidController.reset();
        intakePidController.setTolerance(0);
    }

    @Override
    public void execute() {
        intakePidController.setSetpoint(sp);
        double speed = intakePidController.calculate(AlgaeIntakeSubsystem.getEncoder().get());
            AlgaeIntakeSubsystem.IntakeMoveDrive(speed);
            AlgaeIntakeSubsystem.intakeDrive(AlgaeIntakeConstants.IntakeCollectSpeed);
       
           
        
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
