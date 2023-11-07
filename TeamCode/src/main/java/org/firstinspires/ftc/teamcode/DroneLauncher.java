package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher {
    public Servo trigger;
    public Servo launchPos;

    public boolean launcherReady = false;

    double launchPosition = 0.47;
    double launchStow = 0.96;
    double triggerSafe = 0;
    double triggerRelease = 0.3;

    private Telemetry telemetry;

    public DroneLauncher (HardwareMap hardwareMap, Telemetry telemetry){
        trigger = hardwareMap.get(Servo.class, "droneRelease");
        launchPos = hardwareMap.get(Servo.class, "dronePosition");
        this.telemetry = telemetry;
    }

    public void launch(){
        trigger.setPosition(triggerRelease);
    }

    public void setTriggerSafe(){
        trigger.setPosition(triggerSafe);
    }

    public void readyLaunch(){
        launchPos.setPosition(launchPosition);
        launcherReady = true;
    }

    public void launcherSafe(){
        setTriggerSafe();
        stowLauncher();
        launcherReady = false;
    }

    public void stowLauncher(){
        launchPos.setPosition(launchStow);
        launcherReady = false;
    }

    public void tuneTrigger(int nudge){
        double currentPos = trigger.getPosition();
        if(nudge>0){
            trigger.setPosition(currentPos+0.05);
        } else if(nudge <0){
            trigger.setPosition(currentPos-0.05);
        }
    }

    public void tunePosition(int nudge){
        double currentPos = launchPos.getPosition();
        if(nudge>0){
            launchPos.setPosition(currentPos+0.05);
        } else if(nudge <0){
            launchPos.setPosition(currentPos-0.05);
        }
    }

    public void toggleLauncher(){
        if(launcherReady){
            stowLauncher();
            launcherReady = false;
        } else {
            readyLaunch();
            launcherReady = true;
        }
    }

    public boolean getLauncherReady(){
        return launcherReady;
    }


}
