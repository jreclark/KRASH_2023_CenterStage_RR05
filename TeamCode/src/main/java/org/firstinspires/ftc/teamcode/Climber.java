package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Climber {
    private DcMotorEx leftClimb;
    private DcMotorEx rightClimb;
    Telemetry telemetry;

    private int climberTarget = 500;

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
        leftClimb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightClimb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftClimb.setPower(power);
        rightClimb.setPower(power);
    }

    public double getLeftEncoder(){
        //return leftClimb.getCurrentPosition();
        return leftClimb.getCurrent(CurrentUnit.AMPS);
    }
    public int getRightEncoder(){
        return rightClimb.getCurrentPosition();
    }

    public void runClimberUp(){
        leftClimb.setTargetPosition(climberTarget);
        rightClimb.setTargetPosition(climberTarget);

        leftClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftClimb.setPower(1.0);
        rightClimb.setPower(1.0);

    }
}
