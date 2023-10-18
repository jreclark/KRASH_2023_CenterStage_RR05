package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    public Servo gripper, ejector, swivel;
    public DcMotorEx shoulder;

    private Telemetry telemetry;

    final double GRIPPER_RELEASE = 0;
    final double GRIPPER_RELEASE_ONE = 0;
    final double GRIPPER_HOLD_ALL = 0;

    final double PUSHER_RETRACT = 0;
    final double PUSHER_PUSH1 = 0;
    final double PUSHER_PUSH2 = 0;

    final double ROTATER_PICKUP = 0;
    final double ROTATER_PLACE = 0;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        gripper = hardwareMap.get(Servo.class, "gripper");
        ejector = hardwareMap.get(Servo.class, "ejector");
        swivel = hardwareMap.get(Servo.class, "swivel");
        shoulder = hardwareMap.get(DcMotorEx.class,"shoulder");
    }

    public void setGripper(double pos){
        gripper.setPosition(pos);
    }

    public void setPusher(double pos){
        ejector.setPosition(pos);
    }

    public void setSwivel(double pos) {
        swivel.setPosition(pos);
    }

    public void gripperRelease(){
        setGripper(GRIPPER_RELEASE);
    }

    public void gripperReleaseOne(){
        setGripper(GRIPPER_RELEASE_ONE);
    }

    public void gripperHoldAll(){
        setGripper(GRIPPER_HOLD_ALL);
    }

    public void pusherRetract(){
        setPusher(PUSHER_RETRACT);
    }

    public void pusherPush1(){
        setPusher(PUSHER_PUSH1);
    }

    public void pusherPush2(){
        setPusher(PUSHER_PUSH2);
    }

    public void readyPickup(){
        pusherRetract();
        gripperRelease();
    }

    public void drop1(){
        gripperReleaseOne();
        pusherPush1();
    }

    public void drop2(){
        gripperRelease();
        pusherPush2();
    }

    public void safePosition(){
        gripperRelease();
        pusherRetract();
    }

    public void runShoulder(double power){
        shoulder.setPower(power);
    }

}
