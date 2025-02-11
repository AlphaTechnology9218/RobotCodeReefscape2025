package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class LiveTuning{
    public double Kp, Ki, Kd, p ,i, d;
    String key;
    int currentIndex;
    static int globalIndex = 0;
    static List<LiveTuning> a = new ArrayList<>(10);
    
    public LiveTuning(double Kp, double Ki, double Kd, String key){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.key = key;
        SmartDashboard.putNumber( key + " P", Kp);
        SmartDashboard.putNumber( key + " I", Ki);
        SmartDashboard.putNumber( key + " D", Kd);
        a.add(this);

        currentIndex = globalIndex;
        globalIndex += 1;
    }

    public double getp(){
        return Kp;
    }

    public double geti(){
        return Ki;
    }

    public double getd(){
        return Kd;
    }
    
    public void updatePID(){
        this.p = SmartDashboard.getNumber(key + " P", Kp);
        this.i = SmartDashboard.getNumber(key + " I", Ki);
        this.d = SmartDashboard.getNumber(key + " D", Kd);
        if(p != Kp){
            Kp = p;
        }else if(i != Ki){
            Ki = i;
        }else if(d != Kd){
            Kd = d;
        }

    }
}