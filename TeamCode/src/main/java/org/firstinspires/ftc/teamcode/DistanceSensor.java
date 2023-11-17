package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DriveTrain.HowellMecanumDrive;

/**
 * {@link DistanceSensor} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 * <p>
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 */

public class DistanceSensor {

    private Rev2mDistanceSensor leftRange, rightRange;

    HardwareMap hardwareMap;

    double D_SENSORS = 13.625; //Distance between sensors in inches
    double a, b, robotAngle, robotDistance;

    public DistanceSensor(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        leftRange = hardwareMap.get(Rev2mDistanceSensor.class, "left_range");
        rightRange = hardwareMap.get(Rev2mDistanceSensor.class, "right_range");
    }

    public void updatePosition() {
        a = leftRange.getDistance(DistanceUnit.INCH);
        b = rightRange.getDistance(DistanceUnit.INCH);

        robotDistance = (a + b) / 2;
        robotAngle = Math.toDegrees(Math.asin((b - a) / D_SENSORS));
    }

    public double getRobotAngle() {
        return robotAngle;
    }

    public double getRobotDistance() {
        return robotDistance;
    }

    public double getLeft(){
        return a;
    }

    public double getRight(){
        return b;
    }


}
