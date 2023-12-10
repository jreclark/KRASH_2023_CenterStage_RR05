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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Wing Bonus", group = "Red")
public class RedWing_Bonus extends LinearOpMode {
    //    private ContourTSEProcessor visionProcessor;
    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private FirstVisionProcessor.Selected position = FirstVisionProcessor.Selected.NONE;
    private Robot m_robot;
    private ElapsedTime timer = new ElapsedTime();


    public void runOpMode() throws InterruptedException {
        //        visionProcessor = new ContourTSEProcessor();
        visionProcessor = new FirstVisionProcessor();
//        visionProcessor.setAlliance(ContourTSEProcessor.Alliance.RED);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
        m_robot = new Robot(hardwareMap, telemetry, false);
        m_robot.arm.autoInit();

        double xPlace = 49.5;
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(-90));
        Pose2d leftDeliverPose = new Pose2d(xPlace, -31, Math.toRadians(0));
        Pose2d centDeliverPose = new Pose2d(xPlace, -36, Math.toRadians(0));
        Pose2d rightDeliverPose = new Pose2d(xPlace, -41, Math.toRadians(0));

        m_robot.drive.setPoseEstimate(startPose);

        //Create optional constraints for slower acceleration and speed
        TrajectoryAccelerationConstraint slowAccel = m_robot.drive.getAccelerationConstraint(20);
        TrajectoryVelocityConstraint slowSpeed = m_robot.drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH);

        TrajectorySequence drop = null;
        TrajectorySequence pickup = null;
        TrajectorySequence deliver = null;
        TrajectorySequence deliverGold = null;
        TrajectorySequence park = null;

        //Middle Position Paths
        TrajectorySequence dropCent = m_robot.drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-53, -15, Math.toRadians(-30)), Math.toRadians(90))
                .build();

        TrajectorySequence pickupExtraCent = m_robot.drive.trajectorySequenceBuilder(dropCent.end())
                //.setVelConstraint(slowSpeed)
                .turn(Math.toRadians(-160))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-64, -11, Math.toRadians(-179.99)), Math.toRadians(-179.99))
                .build();

        TrajectorySequence deliverCent = m_robot.drive.trajectorySequenceBuilder(pickupExtraCent.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-46, -12))
                .turn(Math.toRadians(179.99))
                .lineToLinearHeading(new Pose2d(36, -11, Math.toRadians(0)))
                .lineToLinearHeading(leftDeliverPose)  //Drop on the left side
                .build();

        TrajectorySequence deliverGoldCent = m_robot.drive.trajectorySequenceBuilder(deliverCent.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(-179.99))
                .lineToConstantHeading(new Vector2d(44, leftDeliverPose.getY()))
                .lineToConstantHeading(new Vector2d(44, centDeliverPose.getY()))
                .lineToConstantHeading(centDeliverPose.vec())
                .build();

        TrajectorySequence parkCent = m_robot.drive.trajectorySequenceBuilder(deliverGoldCent.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(deliverCent.end().getX()-2, -10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, -10, Math.toRadians(0)))
                .build();


        //Left Position Paths
//        Old Version
//        TrajectorySequence dropLeft = m_robot.drive.trajectorySequenceBuilder(startPose)
//                //.setVelConstraint(slowSpeed)
//                .setTangent(Math.toRadians(90))
//                //.lineToConstantHeading(new Vector2d(-40, -46))
//                .lineToConstantHeading(new Vector2d(-40, -28))
//                .setTangent(Math.toRadians(-179.99))
//                .splineToLinearHeading(new Pose2d(-55, -17, Math.toRadians(-60)), Math.toRadians(90))
//                .build();

        TrajectorySequence dropLeft = m_robot.drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(startPose.getX(), -14))
                .turn(Math.toRadians(-45))
                .build();

        TrajectorySequence pickupExtraLeft = m_robot.drive.trajectorySequenceBuilder(dropLeft.end())
                //.setVelConstraint(slowSpeed)
                .turn(Math.toRadians(-45))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-64, -11), Math.toRadians(-179.99))
                .build();

        TrajectorySequence deliverLeft = m_robot.drive.trajectorySequenceBuilder(pickupExtraLeft.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-46, -12))
                .turn(Math.toRadians(179.99))
                //.lineToLinearHeading(new Pose2d(-30, -11, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(36, -11, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(48, -31.5, Math.toRadians(0)))  //Scoring Position
                .lineToLinearHeading(new Pose2d(xPlace, -40, Math.toRadians(0)))
                .build();

        TrajectorySequence deliverGoldLeft = m_robot.drive.trajectorySequenceBuilder(deliverLeft.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(-179.99))
                .lineToConstantHeading(new Vector2d(44, -40))
                .lineToConstantHeading(new Vector2d(44, -24))
                .lineToConstantHeading(new Vector2d(xPlace, -24))
                .build();


        TrajectorySequence parkLeft = m_robot.drive.trajectorySequenceBuilder(deliverGoldLeft.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(deliverLeft.end().getX()-2, -12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                .build();

        //Right Position Paths
        TrajectorySequence dropRight = m_robot.drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-42, -32, Math.toRadians(0)))
                .build();

        TrajectorySequence pickupExtraRight = m_robot.drive.trajectorySequenceBuilder(dropCent.end())
                //.setVelConstraint(slowSpeed)
                .turn(Math.toRadians(-179.99))
                .setTangent(Math.toRadians(-179.99))
                .splineToLinearHeading(new Pose2d(-50, -20, Math.toRadians(-179.99)), Math.toRadians(-179.99))
                .splineToLinearHeading(new Pose2d(-64, -11, Math.toRadians(-179.99)), Math.toRadians(-179.99))
                .build();

        TrajectorySequence deliverRight = m_robot.drive.trajectorySequenceBuilder(pickupExtraRight.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-46, -12))
                .turn(Math.toRadians(179.99))
                .lineToLinearHeading(new Pose2d(36, -11, Math.toRadians(0)))
                .lineToLinearHeading(leftDeliverPose)  //Drop on the left side
                .build();

        TrajectorySequence deliverGoldRight = m_robot.drive.trajectorySequenceBuilder(deliverRight.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(-179.99))
                .lineToConstantHeading(new Vector2d(44, leftDeliverPose.getY()))
                .lineToConstantHeading(new Vector2d(44, rightDeliverPose.getY()))
                .lineToConstantHeading(rightDeliverPose.vec())
                .build();

        TrajectorySequence parkRight = m_robot.drive.trajectorySequenceBuilder(deliverGoldRight.end())
                //.setVelConstraint(slowSpeed)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(deliverRight.end().getX()-2, -12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))
                .build();


        while ((!isStarted() && !isStopRequested())) {
            telemetry.addData("x = ", visionProcessor.getLocation());
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
                pickup = pickupExtraLeft;
                deliver = deliverLeft;
                deliverGold = deliverGoldLeft;
                park = parkLeft;
                break;
            case RIGHT:
                drop = dropRight;
                pickup = pickupExtraRight;
                deliver = deliverRight;
                deliverGold = deliverGoldRight;
                park = parkRight;
                break;
            default:
                drop = dropCent;
                pickup = pickupExtraCent;
                deliver = deliverCent;
                deliverGold = deliverGoldCent;
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
        m_robot.arm.readyPickup(true);
        sleep(0);  //Delay time here

        //Drive to pickup extra
        m_robot.drive.followTrajectorySequence(pickup);
        timer.reset();
        while (timer.seconds() <= 1.5) {
            m_robot.arm.pickupSequence();
        }
        m_robot.arm.swivelPickup();
        sleep(200);
        m_robot.arm.pickup();
        sleep(200);
        m_robot.arm.readyPickup(true);

        //Deliver to backdrop
        timer.reset();
        m_robot.drive.followTrajectorySequenceAsync(deliver);
        while(m_robot.drive.isBusy()){
            m_robot.drive.update();
            if(timer.seconds()>4.5){
                m_robot.arm.readyDeliverFrontHigh();
            }
        }


        m_robot.arm.drop1();
        sleep(1000);
        //m_robot.arm.runExtensionToPosition(m_robot.arm.EXTENSION_PICKUP);
        //sleep(1000);
        //m_robot.arm.pickupSequence();
        //sleep(1000);

        m_robot.drive.followTrajectorySequence(deliverGold);
        m_robot.arm.drop2();
        sleep(1000);
        m_robot.arm.runExtensionToPosition(m_robot.arm.EXTENSION_PICKUP);
        sleep(2000);


    }
}
