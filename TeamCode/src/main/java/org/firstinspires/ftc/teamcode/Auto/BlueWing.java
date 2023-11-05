package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.DriveTrain.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.DriveTrain.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.processors.ContourTSEProcessor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Blue Wing", group = "Blue")
public class BlueWing extends LinearOpMode {
    private ContourTSEProcessor visionProcessor;
    private VisionPortal visionPortal;
    private ContourTSEProcessor.Selected position = ContourTSEProcessor.Selected.NONE;
    private Robot m_robot;
    private ElapsedTime timer = new ElapsedTime();


    public void runOpMode() throws InterruptedException {
//        visionProcessor = new ContourTSEProcessor();
//        visionProcessor.setAlliance(ContourTSEProcessor.Alliance.BLUE);
//        visionPortal = VisionPortal.easyCreateWithDefaults(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
        m_robot = new Robot(hardwareMap, telemetry, false);
        m_robot.arm.autoInit();

        Pose2d startPose = new Pose2d(-38.5, 62, Math.toRadians(90));
        m_robot.drive.setPoseEstimate(startPose);

        //Create optional constraints for slower acceleration and speed
        TrajectoryAccelerationConstraint slowAccel = m_robot.drive.getAccelerationConstraint(20);
        TrajectoryVelocityConstraint slowSpeed = m_robot.drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH);

        TrajectorySequence drop = null;
        TrajectorySequence deliver = null;

        TrajectorySequence dropCent = m_robot.drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-50, 16, Math.toRadians(45)), Math.toRadians(-90))
                .build();

        TrajectorySequence deliverCent = m_robot.drive.trajectorySequenceBuilder(dropCent.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(-90))
                //.splineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading()
                .splineToLinearHeading(new Pose2d(36, 12, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48,36), Math.toRadians(0))
                .build();

        while ((!isStarted() && !isStopRequested())) {
//            telemetry.addData("x = ", visionProcessor.getSelectedX());
//            telemetry.addData("Selection = ", visionProcessor.getLocation().name());
//            telemetry.update();
        }

        if (isStopRequested()) {
//            visionPortal.stopStreaming();
            return;
        }

//        telemetry.addData("x = ", visionProcessor.getSelectedX());
//        telemetry.update()
//        position = visionProcessor.getSelectedX());
        position = ContourTSEProcessor.Selected.MIDDLE;

        switch (position){
            case LEFT:
                break;
            case RIGHT:
                break;
            default:
                drop = dropCent;
                deliver = deliverCent;
        }

        //Drop the first pixel
        timer.reset();
        m_robot.drive.followTrajectorySequenceAsync(drop);
        while(m_robot.drive.isBusy()){
            m_robot.drive.update();
            if(timer.seconds()>0.5){
                m_robot.arm.pixelSpikeReady();
            }
        }

        m_robot.arm.drop1Flat();
        sleep(500);
        m_robot.arm.gripperHoldAll();
        m_robot.arm.swivelHold();

        //Deliver to backdrop
        timer.reset();
        m_robot.drive.followTrajectorySequenceAsync(deliver);
        while(m_robot.drive.isBusy()){
            m_robot.drive.update();
            if(timer.seconds()>3.0){
                m_robot.arm.readyDeliverFront();
            }
        }

        m_robot.arm.drop2();
        sleep(1000);
        m_robot.arm.readyPickup(false);

    }
}
