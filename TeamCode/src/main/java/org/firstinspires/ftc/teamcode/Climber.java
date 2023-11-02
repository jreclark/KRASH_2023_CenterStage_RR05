package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climber {
    private DcMotorEx leftClimb;
    private DcMotorEx rightClimb;
    Telemetry telemetry;

    public Climber(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftClimb = hardwareMap.get(DcMotorEx.class, "leftClimb");
        rightClimb = hardwareMap.get(DcMotorEx.class, "rightClimb");

        leftClimb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftClimb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightClimb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftClimb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runClimber(double power) {
        leftClimb.setPower(power);
        rightClimb.setPower(power);
    }
}
