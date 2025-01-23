package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

    
public class Limelight extends SubsystemBase {

    double tx = LimelightHelpers.getTX("limelight-alpha");
    double ty = LimelightHelpers.getTY("limelight-alpha");
    double ta = LimelightHelpers.getTA("limelight-alpha");

    LimelightHelpers.LimelightResults aprilResults =
     LimelightHelpers.getLatestResults("limelight-alpha");

    public double getTX(){
        return tx;
    }
    public double getTY(){
        return ty;
    }
    
    public double getTA(){
        return ta;
    }

    @Override
    public void periodic() {
     SmartDashboard.putNumber("LimelightTX", tx);
     SmartDashboard.putNumber("LimelightTY", ty);
     SmartDashboard.putNumber("LimelightTA", ta);
    }


}
