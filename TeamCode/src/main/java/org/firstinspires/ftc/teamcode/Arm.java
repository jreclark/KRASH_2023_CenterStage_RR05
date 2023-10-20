package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.slf4j.helpers.Util;

public class Arm {
    public Servo gripper, ejector, swivel;
    public DcMotorEx shoulder;
    public DcMotorEx extension;
    public Encoder shoulderEncoder;

    private Telemetry telemetry;

    final double GRIPPER_RELEASE = 0;
    final double GRIPPER_RELEASE_ONE = 0.05;
    final double GRIPPER_HOLD_ALL = 0.1;

    final double PUSHER_RETRACT = 1.0;
    final double PUSHER_PUSH1 =0.55;
    final double PUSHER_PUSH2 = 0.75;

    final double ROTATER_PICKUP = 0.18;
    final double ROTATER_PLACE_FRONT = 0.35;
    final double ROTATER_HOLD = 0.85;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        gripper = hardwareMap.get(Servo.class, "gripper");
        ejector = hardwareMap.get(Servo.class, "ejector");
        swivel = hardwareMap.get(Servo.class, "swivel");

        shoulder = hardwareMap.get(DcMotorEx.class,"shoulder");
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extension = hardwareMap.get(DcMotorEx.class,"extension");
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulderEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "shoulder"));
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

    public void swivelPickup(){setSwivel(ROTATER_PICKUP);}

    public void readyPickup(){
        pusherRetract();
        gripperRelease();
        swivelPickup();
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

    public void pickup(){
        setGripper(GRIPPER_HOLD_ALL);
    }

    public void readyDeliverFront(){
        setSwivel(ROTATER_PLACE_FRONT);
    }

    public void wiggle(){
        setSwivel(ROTATER_PICKUP*0.9);
        Utils.sleep(500);
        setSwivel(ROTATER_PICKUP*1.1);
        Utils.sleep(500);
        setSwivel(ROTATER_PICKUP);

    }

    public double getShoulderPosition(){
        return shoulderEncoder.getCurrentPosition();
    }





    public void runShoulder(double power){
        shoulder.setPower(power);
    }

    public void runExtension(double power){
        extension.setPower(power);
    }

    public void runSwivel(double power){
        double currentPos = swivel.getPosition();
        swivel.setPosition(currentPos + power / 100);
    }



}
