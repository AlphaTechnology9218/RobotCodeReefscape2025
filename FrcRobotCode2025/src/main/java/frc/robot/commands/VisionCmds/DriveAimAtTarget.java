package frc.robot.commands.VisionCmds;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveAimAtTarget extends Command{

    SwerveSubsystem swerveSub;
    DoubleSupplier translationx;
    DoubleSupplier translationy;
    double heading;
    double lastGoodHeading;

    public DriveAimAtTarget(SwerveSubsystem swervesub, DoubleSupplier translationx,
     DoubleSupplier translationy){
        this.swerveSub = swervesub;
        this.translationx = translationx;
        this.translationy = translationy;
        lastGoodHeading = 0;
        addRequirements(swervesub);
    }

    @Override
    public void initialize() {
        Optional<Alliance> ally = DriverStation.getAlliance();

        if(ally.isPresent()){
            if(ally.get() == Alliance.Red){
                swerveSub.setVisionTargetID(12);    
            }else if(ally.get() == Alliance.Blue){
                swerveSub.setVisionTargetID(2);
            }
        }
        
    }

    @Override
    public void execute() {
        if(swerveSub.isValidVisionTarget()){
            heading = -swerveSub.getVisionRotationAngle()/70;
        }else{
            System.out.println("Aviso: Mira: Alvo Perdido!");
            RobotContainer.setRightRumbleDriver(1);
            heading = 0;
        }

        lastGoodHeading = heading;
        
        swerveSub.getSwerve().drive(new Translation2d(Math.pow(translationx.getAsDouble(), 3) * swerveSub.getSwerve().getMaximumChassisVelocity(),
        Math.pow(translationy.getAsDouble(), 3) * swerveSub.getSwerve().getMaximumChassisVelocity()),
        heading * swerveSub.getSwerve().getMaximumChassisAngularVelocity(),
        true, false);
    }
    

    @Override
    public void end(boolean interrupted) {
        RobotContainer.setRightRumbleDriver(0);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}