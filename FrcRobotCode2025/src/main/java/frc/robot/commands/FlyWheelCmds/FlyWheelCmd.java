package frc.robot.commands.FlyWheelCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.FlyWheel;



public class FlyWheelCmd extends Command {
   
    FlyWheel flyWheelSubsystem;
    double setpoint;
    

   
 

 public FlyWheelCmd(FlyWheel subsystem, double setpoint) {
        this.flyWheelSubsystem = subsystem;
        this.setpoint = setpoint;
        addRequirements(subsystem);  
 }

    @Override
    public void initialize() {
           
    }
    @Override
    public void execute() {
      setpoint = FlywheelConstants.Setpoint;
      flyWheelSubsystem.FlyWheelActive(setpoint);
      

    }
    @Override
    public void end(boolean interrupted) {
        flyWheelSubsystem.FlyWheelStop();   
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
