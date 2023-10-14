package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.DriveTrain.HowellMecanumDrive;
import org.firstinspires.ftc.teamcode.util.ButtonState;

/**
 * This mode is used to tune servo positions
 */
@TeleOp(group = "Test")
public class ServoTester extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Arm
        Arm arm = new Arm(hardwareMap, telemetry);
        Servo servo = arm.gripper;
        servo.setPosition(0.5);
        int servoSelect = 0;
        int lastServoSelect = 0;

        ButtonState selectUp = new ButtonState(gamepad1, ButtonState.Button.dpad_up);
        ButtonState selectDown = new ButtonState(gamepad1, ButtonState.Button.dpad_down);
        ButtonState posDec = new ButtonState(gamepad1, ButtonState.Button.dpad_left);
        ButtonState posInc = new ButtonState(gamepad1, ButtonState.Button.dpad_right);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Press d-pad Left/right to select a servo.");
            telemetry.addLine("On selecting a servo, it will default to pos = 0.5.");
            telemetry.addLine("Press d-pad Left/Right to move servo by 0.05");
            telemetry.addLine();
            telemetry.addLine("Current Servo: " + servo.getDeviceName());
            telemetry.addLine("Current Position: " + servo.getPosition());

            if (selectUp.newPress()) {
                if (servoSelect == 2) {
                    servoSelect = 0;
                } else {
                    servoSelect++;
                }
            }
            if (selectDown.newPress()){
                if(servoSelect ==0){
                    servoSelect = 2;}
                else{
                    servoSelect --;
                }
            }
            if(posInc.newPress()){
                servo.setPosition(servo.getPosition() + 0.05);
            }
            if(posDec.newPress()){
                servo.setPosition(servo.getPosition() - 0.05);
            }

            switch(servoSelect){
                case 0:
                    servo = arm.gripper;
                    break;
                case 1:
                    servo = arm.pusher;
                    break;
                case 2:
                    servo = arm.rotater;
                    break;
            }

            if(lastServoSelect != servoSelect) {servo.setPosition(0.5);}

            telemetry.update();

        }
    }
}
