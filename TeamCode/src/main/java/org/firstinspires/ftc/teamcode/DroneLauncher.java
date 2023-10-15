package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher {
    private Servo droneServo;
    private Telemetry telemetry;

    public DroneLauncher (HardwareMap hardwareMap, Telemetry telemetry){
        droneServo = hardwareMap.get(Servo.class, "droneServo");
        this.telemetry = telemetry;
    }

    public void launch(){
        droneServo.setPosition(0.7);
    }


}
