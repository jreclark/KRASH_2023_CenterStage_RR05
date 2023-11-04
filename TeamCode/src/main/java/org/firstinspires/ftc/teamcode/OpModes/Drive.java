package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.DriveTrain.DriveConstants.MAX_VEL;

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
    public boolean armManual = true;

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

        //m_robot.drive.setMotorPowers(-.5,-.5,-.5,-.5); // Set motor power to 50%???

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
                            Math.pow(input.getX(),3),
                            Math.pow(input.getY(),3),
                            Math.pow(-gamepad1.right_stick_x,3)
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
            telemetry.addData("Arm position", m_robot.arm.getShoulderPosition());
            telemetry.addData("Extension position", m_robot.arm.getExtensionPosition());
            telemetry.addData("Swivel position", m_robot.arm.swivel.getPosition());


            if(gamepad1.right_bumper) {fieldRel = true;}
            if(gamepad1.left_bumper) {fieldRel = false;}
            if(gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5) {
                m_robot.drive.resetGyro();
            }

            //ARM Controls
            if(Math.abs(gamepad2.left_stick_x)>0.1) {m_robot.arm.runSwivel(gamepad2.left_stick_x);}
            if(armManual) {
                m_robot.arm.runShoulder(gamepad2.right_stick_x * 1.0);
                m_robot.arm.runExtension(-gamepad2.left_stick_y * 0.5);
            }

            if(Math.abs(gamepad2.right_stick_x)>0.1 || Math.abs(gamepad2.left_stick_y) > 0){
                armManual = true;
            }

            //D-Pad Definitions
            if(gamepad2.dpad_down){
                armManual = false;
                m_robot.arm.readyPickup();
            }

            if(gamepad2.dpad_right){
                armManual = false;
                m_robot.arm.readyDeliverFront();
            }
            if(gamepad2.dpad_up){
                armManual = false;
                m_robot.arm.readyDeliverBackHigh();
            }
//            if(gamepad2.dpad_left){
//                m_robot.arm.readyDeliverBackLow();
//            }

            //Trigger / Shoulder Controls
            if(gamepad2.right_trigger >= 0.5){
                armManual = false;
                m_robot.arm.pickup();
            }
            if(gamepad2.left_trigger >= 0.2){
                armManual = false;
                m_robot.arm.pickupSequence();
            } else {
                if (m_robot.arm.getInPickupSequencec()) {
                    m_robot.arm.swivelPickup();
                }
                m_robot.arm.clearInPickupSequence();
            }

            if(gamepad2.right_bumper){
                m_robot.arm.drop1();
            }
            if(gamepad2.left_bumper){
                m_robot.arm.drop2();
            }

            //Button Controls
            if(gamepad2.a){
                m_robot.arm.wiggle();
            }
            if(gamepad2.y){
                m_robot.arm.swivelHold();
            }

            //Climber controls
            if(gamepad1.x){
                m_robot.climber.runClimber(0.5);
            } else if(gamepad1.b){
                m_robot.climber.runClimber(-0.5);
            } else m_robot.climber.runClimber(0);

            telemetry.update();


        }
    }
}
