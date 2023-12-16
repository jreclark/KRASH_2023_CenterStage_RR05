package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.DriveTrain.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.DriveTrain.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.processors.ContourTSEProcessor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Blue Backstage", group = "Blue")
public class BlueBackstage extends LinearOpMode {
    private ContourTSEProcessor visionProcessor;
    private VisionPortal visionPortal;
    private ContourTSEProcessor.Selected position = ContourTSEProcessor.Selected.NONE;
    private Robot m_robot;
    private ElapsedTime timer = new ElapsedTime();


    public void runOpMode() throws InterruptedException {
        visionProcessor = new ContourTSEProcessor();
        visionProcessor.setAlliance(ContourTSEProcessor.Alliance.BLUE);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
        m_robot = new Robot(hardwareMap, telemetry, false);
        m_robot.arm.autoInit();

        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(90));
        m_robot.drive.setPoseEstimate(startPose);

        //Create optional constraints for slower acceleration and speed
        TrajectoryAccelerationConstraint slowAccel = m_robot.drive.getAccelerationConstraint(20);
        TrajectoryVelocityConstraint slowSpeed = m_robot.drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH);

        TrajectorySequence drop = null;
        TrajectorySequence deliver = null;
        TrajectorySequence park = null;

        //Middle Position Paths
        TrajectorySequence dropCent = m_robot.drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(32, 24, Math.toRadians(179)), Math.toRadians(-90))
                .build();

        TrajectorySequence deliverCent = m_robot.drive.trajectorySequenceBuilder(dropCent.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(47.5, 33, Math.toRadians(0)))
                .build();

        TrajectorySequence parkCent = m_robot.drive.trajectorySequenceBuilder(deliverCent.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(deliverCent.end().getX()-2, 60, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, 62, Math.toRadians(0)))
                .build();

        //Left Position Paths
        TrajectorySequence dropLeft = m_robot.drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(40, 32, Math.toRadians(179)))
                .build();

        TrajectorySequence deliverLeft = m_robot.drive.trajectorySequenceBuilder(dropLeft.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(90))
                .turn(-Math.toRadians(179.0))
                .lineToLinearHeading(new Pose2d(48, 44, Math.toRadians(0)))
                .build();

        TrajectorySequence parkLeft = m_robot.drive.trajectorySequenceBuilder(deliverLeft.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(deliverCent.end().getX()-2, 60, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, 62, Math.toRadians(0)))
                .build();

        //Right Position Paths
        TrajectorySequence dropRight = m_robot.drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(18, 32, Math.toRadians(179)))
                .build();

        TrajectorySequence deliverRight = m_robot.drive.trajectorySequenceBuilder(dropRight.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(48, 27, Math.toRadians(0)))
                .build();

        TrajectorySequence parkRight = m_robot.drive.trajectorySequenceBuilder(deliverRight.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(deliverCent.end().getX()-2, 60, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, 62, Math.toRadians(0)))
                .build();


        while ((!isStarted() && !isStopRequested())) {
            telemetry.addData("x = ", visionProcessor.getSelectedX());
            telemetry.addData("Selection = ", visionProcessor.getLocation().name());
            telemetry.update();
        }

        if (isStopRequested()) {
            visionPortal.stopStreaming();
            return;
        }

        telemetry.addData("x = ", visionProcessor.getLocation());
        telemetry.update();
        position = visionProcessor.getLocation();
//        position = ContourTSEProcessor.Selected.LEFT;

        switch (position){
            case LEFT:
                drop = dropLeft;
                deliver = deliverLeft;
                park = parkLeft;
                break;
            case RIGHT:
                drop = dropRight;
                deliver = deliverRight;
                park = parkRight;
                break;
            default:
                drop = dropCent;
                deliver = deliverCent;
                park = parkCent;
        }

        //Drop the first pixel
        timer.reset();
        m_robot.drive.followTrajectorySequenceAsync(drop);
        while(m_robot.drive.isBusy()){
            m_robot.drive.update();
            if(timer.seconds()>0.5){
                m_robot.arm.swivelPickup();
            }
        }
        m_robot.arm.pixelSpikeReady();
        sleep(1200);
        m_robot.arm.drop1Flat();
        sleep(750);
        m_robot.arm.gripperHoldAll();
        m_robot.arm.pixelHold();
        sleep(500);

        //Deliver to backdrop
        timer.reset();
        m_robot.drive.followTrajectorySequenceAsync(deliver);
        while(m_robot.drive.isBusy()){
            m_robot.drive.update();
            if(timer.seconds()>0){
                m_robot.arm.readyDeliverFront(false);
            }
        }

        //Did not fully extend because we hit the backdrop
        m_robot.arm.runExtensionToPosition(m_robot.arm.EXTENSION_DELIVER_FRONT);
        sleep(500);

        m_robot.arm.drop2();
        sleep(1000);
        m_robot.arm.runExtensionToPosition(m_robot.arm.EXTENSION_PICKUP);
        sleep(1000);
        m_robot.arm.readyPickup(false);
        sleep(1000);

        m_robot.drive.followTrajectorySequence(park);

    }
}
