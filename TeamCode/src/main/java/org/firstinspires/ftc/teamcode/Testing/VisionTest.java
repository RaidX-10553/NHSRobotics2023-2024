package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Testing.ColorDetector.Detection;
import static org.firstinspires.ftc.teamcode.Testing.ColorDetector.TargetColor;
import static org.firstinspires.ftc.teamcode.Testing.ColorDetector.ViewMode;
import static org.firstinspires.ftc.teamcode.Testing.ColorDetector.centerRect;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.Testing.ColorDetector;
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
        while (!detector.isDetectionConfident()){
            switch (detector.getConfidentDetection()) {
                case LEFT:
                    telemetry.addLine("left");
                    telemetry.update();
                    zone = 1;
                    break;
                case MIDDLE:
                    telemetry.addLine("mid");
                    telemetry.update();
                    zone = 2;
                    break;
                case NONE:
                    telemetry.addLine("right");
                    telemetry.update();
                    zone = 3;
                    break;
                default:
                    telemetry.addLine("none");
                    telemetry.update();
                    zone = 4;
                    break;
            }
        }
        telemetry.addLine("been detected");
        telemetry.update();


        if(zone==1) {
            telemetry.addLine("left for real");
            telemetry.update();
        }
        if(zone==3) {
            telemetry.addLine("right for real");
            telemetry.update();
        }
        if(zone==2) {
            telemetry.addLine("mid af");
            telemetry.update();
        }
        if(zone==4) {
            telemetry.addLine("we messed up big time");
            telemetry.update();
        }


    }
}
