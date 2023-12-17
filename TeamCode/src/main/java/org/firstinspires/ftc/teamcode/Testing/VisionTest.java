package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.subsystem.ColorDetector.Detection;
import static org.firstinspires.ftc.teamcode.subsystem.ColorDetector.TargetColor;
import static org.firstinspires.ftc.teamcode.subsystem.ColorDetector.ViewMode;
import static org.firstinspires.ftc.teamcode.subsystem.ColorDetector.centerRect;


import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.subsystem.ColorDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;


@Config
@Autonomous(name="VisionTest", group="Autonomous")
public class VisionTest extends OpMode{

    ColorDetector detector;

    VisionPortal visionPortal;


    public int zone = 0;

    @Override
    public void init() {

        Rect leftZone = centerRect(240,240,150,300);
        Rect midZone = centerRect(560,180,150,200);

        detector = new ColorDetector(telemetry, TargetColor.RED, ViewMode.RAW, leftZone, midZone);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), detector);
        CameraStreamServer.getInstance().setSource(detector);

        telemetry.addData("Detection captured:",detector.getConfidentDetection());
        Detection detection = detector.getConfidentDetection();






    }


    @Override
    public void loop() {

        if(zone == 1){
            telemetry.addLine("going left");
            telemetry.update();



        }
        if(zone == 0){
            telemetry.addLine("going right");
            telemetry.update();


        }
        if(zone == 3){
            telemetry.addLine("going mid");
            telemetry.update();


        }

        if(detector.getConfidentDetection() == Detection.LEFT) {
            telemetry.addLine("left for real");
            zone = 1;
            telemetry.update();

        }
        if(detector.getConfidentDetection() == Detection.RIGHT) {
            telemetry.addLine("right for real");
            zone = 0;
            telemetry.update();

        }
        if(detector.getConfidentDetection() == Detection.MIDDLE) {
            telemetry.addLine("mid af");
            zone = 3;
            telemetry.update();

        }





    }
}
