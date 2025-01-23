/*package frc.robot.commands.ArmCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDrive extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ArmSubsystem arm;
    private Supplier<Double> speed;
    private double sp;
    

    public ArmDrive(ArmSubsystem subsystem, Supplier<Double> speed){
        this.arm = subsystem;
        this.speed = speed;
        addRequirements(subsystem);

    }

    @Override
    public void initialize(){
        
        
    }
 
    @Override
    public void execute(){
        sp = arm.getAbsEncoder().getAbsolutePosition() - arm.getAbsEncoder().getPositionOffset();
        if(sp > 0.8){
            arm.armDrive(0.4);
        }else{
            arm.armDrive(speed.get());
        }
         
    }

    @Override
    public void end(boolean interrupted){
        arm.stopArm();
    }

    @Override
    public boolean isFinished(){
        return false;  
    }
}*/
