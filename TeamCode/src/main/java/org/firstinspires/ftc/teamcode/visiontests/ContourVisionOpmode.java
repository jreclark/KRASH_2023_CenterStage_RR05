package org.firstinspires.ftc.teamcode.visiontests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.ContourTSEProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class ContourVisionOpmode extends OpMode {
    private ContourTSEProcessor visionProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        visionProcessor = new ContourTSEProcessor();
        visionProcessor.setAlliance(ContourTSEProcessor.Alliance.BLUE);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
    }

    @Override
    public void init_loop() {

        telemetry.addData("x = ", visionProcessor.getSelectedX());
        telemetry.addData("Selection = ", visionProcessor.getLocation().name());
    }

    @Override
    public void start() {
        visionPortal.saveNextFrameRaw("FinalFrame");
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
        telemetry.addData("x = ", visionProcessor.getSelectedX());
    }
}
