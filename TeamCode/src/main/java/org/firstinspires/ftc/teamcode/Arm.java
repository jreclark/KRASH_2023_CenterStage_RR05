package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.slf4j.helpers.Util;

public class Arm {
    public Servo gripper, ejector, swivel;
    public DcMotorEx shoulder;
    public DcMotorEx extension;
    public Encoder shoulderEncoder;
    public Encoder extensionEncoder;

    private Telemetry telemetry;

    final double GRIPPER_RELEASE = 0;
    final double GRIPPER_RELEASE_ONE = 0.05;
    final double GRIPPER_HOLD_ALL = 0.1;

    final double WIGGLE_WIGGLE = 0.02; //was +/-0.07
    final long WIGGLE_DELAY = 500;

    final double PUSHER_RETRACT = 0; //1.0
    final double PUSHER_PUSH1 =0.65;
    final double PUSHER_PUSH2 = 0.55;

    final double SWIVEL_PICKUP = 0.18;
    final double SWIVEL_PLACE_FRONT = 0.233;
    final double SWIVEL_HOLD = 0.85;
    final double SWIVEL_BACK_LOW = 0.61;
    final double SWIVEL_BACK_HIGH = 0.71;

    final int ARM_MIN = 0;
    final int ARM_MIN_SLOW = 400;
    final int ARM_MAX_SLOW = 5300;
    final int ARM_MAX = 6500;

    final int EXTENSION_MIN = 0;
    final int EXTENSION_MIN_SLOW = 75;
    final int EXTENSION_MAX_SLOW = 425;
    final int EXTENSION_MAX = 450;
    
    final int ARM_READY_PICKUP = 350;
    final int EXTENSION_READY_PICKUP = 40;
    
    final int ARM_PICKUP = 0;
    final int EXTENSION_PICKUP = 75;

    boolean inPickupSequence = false;
    ElapsedTime timer = new ElapsedTime();
    
    
    public enum Arm_Modes {
        PICKUP,
        HOLD,
        DELIVER_FRONT,
        DELIVER_BACK_LOW,
        DELIVER_BACK_HIGH,
        MANUAL
    }

    public Arm(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        gripper = hardwareMap.get(Servo.class, "gripper");
        ejector = hardwareMap.get(Servo.class, "ejector");
        swivel = hardwareMap.get(Servo.class, "swivel");

        shoulder = hardwareMap.get(DcMotorEx.class,"shoulder");
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extension = hardwareMap.get(DcMotorEx.class,"extension");
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulderEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "shoulder"));
        extensionEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "extension"));
        extensionEncoder.setDirection(Encoder.Direction.REVERSE);
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

    public void swivelPickup(){setSwivel(SWIVEL_PICKUP);}
    public void swivelHold(){setSwivel(SWIVEL_HOLD);}
    public void readyDeliverBackHigh(){setSwivel(SWIVEL_BACK_HIGH);}
    public void readyDeliverBackLow(){setSwivel(SWIVEL_BACK_LOW);}
    public void readyDeliverFront(){setSwivel(SWIVEL_PLACE_FRONT);}

    public void readyPickup(){
        runShoulderToPosition(ARM_READY_PICKUP);
        runExtensionToPosition(EXTENSION_READY_PICKUP);
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
        gripperHoldAll();
    }

    public void wiggle(){
        double currentPos = swivel.getPosition();
        setSwivel(currentPos-WIGGLE_WIGGLE);
        Utils.sleep(WIGGLE_DELAY);
        setSwivel(currentPos+WIGGLE_WIGGLE);
        Utils.sleep(WIGGLE_DELAY);
        setSwivel(currentPos);
    }

    public double getShoulderPosition(){
        return shoulderEncoder.getCurrentPosition();
    }

    public double getExtensionPosition(){
        return extensionEncoder.getCurrentPosition();
    }

    public void runShoulder(double power){
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setPower(limitPower(power, ARM_MIN, ARM_MIN_SLOW, ARM_MAX, ARM_MAX_SLOW, shoulderEncoder));
    }

    public void runExtension(double power){
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setPower(limitPower(power, EXTENSION_MIN, EXTENSION_MIN_SLOW, EXTENSION_MAX, EXTENSION_MAX_SLOW, extensionEncoder));
    }

    public void runSwivel(double power){
        double currentPos = swivel.getPosition();
        swivel.setPosition(currentPos + power / 100);
    }

    public double limitPower(double power, int min, int minSlow, int max, int maxSlow, Encoder encoder){
        double slow = 0.3;
        int pos = encoder.getCurrentPosition();
//        if(isInRange(pos, minSlow, maxSlow)){
//            return power;
//        } else
        if ((power >= 0) && isInRange(pos, maxSlow, max)){
            return power*slow;
        } else if ((power >=0) && (pos >= max)){
            return 0;
        } else if ((power < 0) && isInRange(pos, min, minSlow)){
            return power*slow;
        } else if ((power < 0) && (pos <= min)){
            return 0;
        } else {
            return power;
        }
    }

    public boolean isInRange(int val, int min, int max){
        return (val >= min) && (val <= max);
    }
    
    public void runShoulderToPosition(int target){
        shoulder.setTargetPosition(target);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(1.0);        
    }

    public void runExtensionToPosition(int target){
        extension.setTargetPosition(target);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setPower(1.0);
    }

    public void setInPickupSequence(){
        inPickupSequence = true;
    }

    public void clearInPickupSequence(){
        inPickupSequence = false;
    }

    public void pickupSequence(){
        double WIGGLE_HOLD = 0.5;
        if (!inPickupSequence){
            setInPickupSequence();
            timer.reset();
            runShoulderToPosition(ARM_PICKUP);
            runExtensionToPosition(EXTENSION_PICKUP);
            swivelPickup();
        } else {
            double time = timer.seconds();
            if(time>WIGGLE_HOLD){
                double wiggleCount = Math.floor((time - WIGGLE_HOLD) / WIGGLE_DELAY);
                if(wiggleCount % 2 == 0){
                    setSwivel(SWIVEL_PICKUP-WIGGLE_WIGGLE);
                } else {
                    setSwivel(SWIVEL_PICKUP+WIGGLE_WIGGLE);
                }
            }
        }
    }

}
