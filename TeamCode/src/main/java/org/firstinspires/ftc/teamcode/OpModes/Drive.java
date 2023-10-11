package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.HowellMecanumDrive;

/**

 */
@TeleOp(group = "Comp")
public class Drive extends LinearOpMode {
    public boolean fieldRel = true;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize MecanumDrive
        //TODO Need to add pose transfer
        HowellMecanumDrive drive = new HowellMecanumDrive(hardwareMap, telemetry);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
            double poseHeading = drive.getExternalHeading();
            if (!fieldRel){ poseHeading = 0;}

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseHeading);

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", drive.getExternalHeading());
            telemetry.addData("Field Rel", fieldRel);
            telemetry.update();

            if(gamepad1.right_bumper) {fieldRel = true;}
            if(gamepad1.left_bumper) {fieldRel = false;}
            if(gamepad1.x) {
                Pose2d pose = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), 0));
            }
        }
    }
}
