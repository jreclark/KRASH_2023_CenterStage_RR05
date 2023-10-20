package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DriveTrain.HowellMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;

/**

 */
@TeleOp(group = "Comp")
public class Drive extends LinearOpMode {
    public boolean fieldRel = true;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Robot
        Robot m_robot = new Robot(hardwareMap, telemetry, true);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        m_robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = m_robot.drive.getPoseEstimate();
            double poseHeading = m_robot.drive.getExternalHeading();  //Use the gyro instead of odometry
            if (!fieldRel){ poseHeading = 0;}

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseHeading);

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            m_robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            m_robot.drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", m_robot.drive.getExternalHeading());
            telemetry.addData("Field Rel", fieldRel);
            telemetry.addLine();
            telemetry.addData("arm position", m_robot.arm.getShoulderPosition());
            telemetry.update();

            if(gamepad1.right_bumper) {fieldRel = true;}
            if(gamepad1.left_bumper) {fieldRel = false;}
            if(gamepad1.x) {
                m_robot.drive.resetGyro();
            }

            m_robot.arm.runShoulder(-gamepad2.right_stick_y*0.75);
            m_robot.arm.runExtension(gamepad2.left_stick_y*0.5);
            if(Math.abs(gamepad2.right_stick_x)>0.1){m_robot.arm.runSwivel(gamepad2.right_stick_x);}

            if(gamepad2.dpad_down){
                m_robot.arm.readyPickup();
            }

            if(gamepad2.left_trigger >= 0.5){
                m_robot.arm.pickup();
            }

            if(gamepad2.dpad_up){
                m_robot.arm.readyDeliverFront();
            }

            if(gamepad2.right_bumper){
                m_robot.arm.drop1();
            }

            if(gamepad2.left_bumper){
                m_robot.arm.drop2();
            }

            if(gamepad2.a){
                m_robot.arm.wiggle();
            }
        }
    }
}
