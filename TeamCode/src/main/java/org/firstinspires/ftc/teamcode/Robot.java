package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drive.HowellMecanumDrive;

/**
 * Robot class should contain all of your robot subsystems.
 * This class can then be instanced in all of your teleOp and Auton
 * classes to give consistent behavior across all of them.
 */
public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public HowellMecanumDrive drive;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean isTele) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        drive = new HowellMecanumDrive(hardwareMap, telemetry);
    }



}
