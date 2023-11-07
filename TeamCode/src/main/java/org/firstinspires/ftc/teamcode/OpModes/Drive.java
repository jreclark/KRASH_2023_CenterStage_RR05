package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.DriveTrain.DriveConstants.MAX_VEL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DriveTrain.HowellMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.ButtonState;

/**

 */
@TeleOp(group = "Comp")
public class Drive extends LinearOpMode {
    public boolean fieldRel = false;
    public boolean armManual = true;
    public boolean backoffButton = false;
    public boolean launchPosButton = false;

    final double NORMAL_SPEED = 0.8;
    final double SLOW_SPEED = 0.4;
    final double TURBO_SPEED = 1.0;
    public double speedScale = NORMAL_SPEED;
    public double finalScale = speedScale;

    private ButtonState dropButton = new ButtonState(gamepad2, ButtonState.Button.right_bumper);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Robot
        Robot m_robot = new Robot(hardwareMap, telemetry, true);
        m_robot.droneLauncher.launcherSafe();

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
            //if(gamepad1.dpad_right) {fieldRel = true;}
            if(gamepad1.dpad_left) {fieldRel = false;}
            if(gamepad1.b && gamepad1.x) {
                m_robot.drive.resetGyro();
            }
            if(gamepad1.right_bumper) {
                speedScale = NORMAL_SPEED;
            } else if (gamepad1.left_bumper) {
                speedScale = SLOW_SPEED;
            }
            if(gamepad1.right_trigger > 0.5){
                finalScale = TURBO_SPEED;
            } else {
                finalScale = speedScale;
            }





            // Read pose
            Pose2d poseEstimate = m_robot.drive.getPoseEstimate();
            double poseHeading = m_robot.drive.getRawExternalHeading();  //Use the gyro instead of odometry
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
                    ).times(finalScale)
            );

            // Update everything. Odometry. Etc.
            m_robot.drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", m_robot.drive.getRawExternalHeading());
            telemetry.addData("Field Rel", fieldRel);
            telemetry.addLine();
            telemetry.addData("Arm position", m_robot.arm.getShoulderPosition());
            telemetry.addData("Extension position", m_robot.arm.getExtensionPosition());
            telemetry.addData("Swivel position", m_robot.arm.swivel.getPosition());
            telemetry.addData("haveTwo", m_robot.arm.haveTwo);
            telemetry.addLine();
            telemetry.addData("Launcher position", m_robot.droneLauncher.launchPos.getPosition());
            telemetry.addData("Trigger position", m_robot.droneLauncher.trigger.getPosition());


            //ARM Controls
            if(Math.abs(gamepad2.left_stick_x)>0.1) {m_robot.arm.runSwivel(gamepad2.left_stick_x);}
            if(armManual) {
                m_robot.arm.runShoulder(gamepad2.right_stick_x * 1.0);
                m_robot.arm.runExtension(-gamepad2.left_stick_y * 0.5);
            }

            if(Math.abs(gamepad2.right_stick_x)>0.1 || Math.abs(gamepad2.left_stick_y) > 0.1){
                armManual = true;
            }

            //D-Pad Definitions
            if(gamepad2.dpad_down){
                armManual = false;
                m_robot.arm.readyPickup(false);
            }

            if(gamepad2.dpad_right){
                armManual = false;
                m_robot.arm.readyDeliverFront();
            }
            if(gamepad2.dpad_up){
                armManual = false;
                m_robot.arm.readyDeliverBackHigh();
            }
            if(gamepad2.start){
                m_robot.arm.zeroShoulder();
            } else if (m_robot.arm.zeroingArm){
                m_robot.arm.zeroingArm = false;
                m_robot.arm.resetShoulderEncoder();
            }

            //Trigger / Shoulder Controls
//            if(gamepad2.right_trigger >= 0.5){
//                armManual = false;
//                m_robot.arm.pickup();
//            }
            if(gamepad2.left_trigger >= 0.2){
                armManual = false;
                m_robot.arm.pickupSequence();
            } else {
                if (m_robot.arm.getInPickupSequencec()) {
                    m_robot.arm.swivelPickup();
                    sleep(200);
                    m_robot.arm.pickup();
                    sleep(200); //Sleep in tele is usually a horrible idea, but this enforces a pause
                    m_robot.arm.readyPickup(true);
                }
                m_robot.arm.clearInPickupSequence();
            }

            if(gamepad2.right_bumper){
                m_robot.arm.drop1();
                m_robot.arm.clearHaveTwo();
            } else if (gamepad2.left_bumper && !m_robot.arm.haveTwo){
                m_robot.arm.drop2();
            }

            //Button Controls
            if(gamepad2.a){
                m_robot.arm.wiggle();
            }
            if(gamepad2.y){
                m_robot.arm.swivelHold();
            }
            if(gamepad2.x){
                if(!backoffButton) {
                    backoffButton = true;
                    m_robot.arm.backoffShoulder();
                }
            } else {
                backoffButton = false;
            }

            //Climber controls
            if(gamepad1.dpad_up){
                m_robot.climber.runClimber(0.5);
            } else if(gamepad1.dpad_down){
                m_robot.climber.runClimber(-0.5);
            } else m_robot.climber.runClimber(0);

            telemetry.update();


            //Drone controls
            if(gamepad2.b && !launchPosButton){
                m_robot.droneLauncher.toggleLauncher();
                launchPosButton = true;
            } else if (!gamepad2.b){
                launchPosButton = false;
            }

            if(m_robot.droneLauncher.getLauncherReady() && gamepad2.right_trigger >0.5){
                m_robot.droneLauncher.launch();
            } else {
                m_robot.droneLauncher.setTriggerSafe();
            }


            //Drone Fine Tuning
            if(gamepad1.x){
                m_robot.droneLauncher.tunePosition(-1);
            } else if(gamepad1.b){
                m_robot.droneLauncher.tunePosition(1);
            }
        }
    }
}
