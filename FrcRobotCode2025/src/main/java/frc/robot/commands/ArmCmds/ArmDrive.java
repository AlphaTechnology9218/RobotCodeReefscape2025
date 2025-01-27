package frc.robot.commands.ArmCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmDrive extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private Arm arm;
    private Supplier<Double> speed, speed2;
   
    

    public ArmDrive(Arm subsystem, Supplier<Double> speed, Supplier<Double> speed2){
        this.arm = subsystem;
        this.speed = speed;
        this.speed2 = speed2;
        addRequirements(subsystem);

    }

    @Override
    public void initialize(){
        
        
    }
 
    @Override
    public void execute(){
        arm.ShouderDrive(speed.get());
        arm.WristDrive(speed2.get());
        
         
    }

    @Override
    public void end(boolean interrupted){
        arm.stopArm();
    }

    @Override
    public boolean isFinished(){
        return false;  
    }
}
