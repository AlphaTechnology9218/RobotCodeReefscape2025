package frc.robot.commands.ArmCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmDrive extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private Arm arm;
    private Supplier<Double> speed;
   
    

    public ArmDrive(Arm subsystem, Supplier<Double> speed){
        this.arm = subsystem;
        this.speed = speed;
        addRequirements(subsystem);

    }

    @Override
    public void initialize(){
        
        
    }
 
    @Override
    public void execute(){
        arm.armDrive(speed.get());
        
         
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
