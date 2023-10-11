package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * The Drive class holds the elements basic wheel/motor controls
 */
public class Drive {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DcMotorEx leftFront, rightFront, leftRear, rightRear;

    public static final double WHEEL_DIAMETER = 4;  //Wheel diameter in inches
    public static final double GEAR_RATIO = 19.2;  //Gearbox ration on motor
    public static final int TICKS_PER_REV = 28; // Encoder ticks per motor revolution

    public static final double inchesPerWheelRev = Math.PI * WHEEL_DIAMETER;
    public static final double inchesPerMotorRev = inchesPerWheelRev / GEAR_RATIO;
    public static final double ticksPerInch = TICKS_PER_REV / inchesPerMotorRev;


    public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Assign deviceNames from the phone HardwareMap to the motor variables
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Reverse motor directions on one side of the robot to account for motor orientation
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initially set all motors to the most basic run mode.
        // RUN_WITHOUT_ENCODER also results in the maximum available motor speed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to Brake mode when there is no motor input.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Simple method to batch-set all four motor powers at once.
     * @param leftFrontPwr
     * @param leftRearPwr
     * @param rightFrontPwr
     * @param rightRearPwr
     */
    public void setMotorPowers(double leftFrontPwr, double leftRearPwr,
                               double rightRearPwr, double rightFrontPwr) {

        leftFront.setPower(leftFrontPwr);
        leftRear.setPower(leftRearPwr);
        rightFront.setPower(rightFrontPwr);
        rightRear.setPower(rightRearPwr);
    }

    /**
     * Immediately stop all drive motors
     */
    public void stopMotors(){
        setMotorPowers(0, 0, 0, 0);
    }

    /**
     * Batch set drive motor RunMode
     * @param runMode
     */
    public void setMotorsMode(DcMotor.RunMode runMode){
        leftFront.setMode(runMode);
        leftRear.setMode(runMode);
        rightFront.setMode(runMode);
        rightRear.setMode(runMode);
    }


    /**
     * Simple drive to position example using motor encoders.
     * Use + power to drive forward and - power to drive backward.
     *
     * @param inches
     * @param power
     * @param timeout
     * @return
     */
    public boolean driveByEncodersTest(double inches, double power, double timeout){
        ElapsedTime driveTime = new ElapsedTime();

        int targetCounts = (int)(Math.round(inches * ticksPerInch) * Math.signum(power));

        telemetry.addData("Target: ", targetCounts);
        telemetry.update();
        Utils.sleep(5000);

        leftFront.setTargetPosition(targetCounts);
        leftRear.setTargetPosition(targetCounts);
        rightFront.setTargetPosition(targetCounts);
        rightRear.setTargetPosition(targetCounts);

        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPowers(power, power, power, power);
        driveTime.reset();

        while(driveIsBusy() && driveTime.seconds() < timeout){
            telemetry.addData("LeftFrontCounts: ", leftFront.getCurrentPosition());
            telemetry.update();
        }

        boolean finished = !driveIsBusy();
        stopMotors();

        return finished;
    }

    public boolean driveIsBusy(){
        return leftFront.isBusy() ||
                leftRear.isBusy() ||
                rightFront.isBusy() ||
                rightRear.isBusy();
    }

}
