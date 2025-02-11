package frc.robot.commands.IntakeCmds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.AlgaeIntake;

public class IntakeAlgaHoldCmd extends Command{
    
    AlgaeIntake AlgaeIntakeSubsystem;
    private double sp ,spRealease;
    private PIDController intakePidController = new PIDController(AlgaeIntakeConstants.kP,
    AlgaeIntakeConstants.kI, AlgaeIntakeConstants.kD);

    public IntakeAlgaHoldCmd(AlgaeIntake subsystem){
        this.AlgaeIntakeSubsystem = subsystem; 
        this.spRealease = AlgaeIntakeConstants.spRealease; 
        this.sp = spRealease;
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
