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

    public enum Zone {
        LEFT,
        RIGHT,
        MID,
        NONE,
    }

    public Zone zone;


    @Override
    public void init() {

        Rect leftZone = centerRect(240,240,150,300);
        Rect midZone = centerRect(560,180,150,200);

        detector = new ColorDetector(telemetry, TargetColor.RED, ViewMode.RAW, leftZone, midZone);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), detector);
        CameraStreamServer.getInstance().setSource(detector);

        telemetry.addData("Detection captured:",detector.getConfidentDetection());






    }
    @Override
    public void init_loop() {
        switch (detector.getConfidentDetection()) {
            case LEFT:
                telemetry.addLine("left");
                telemetry.update();
                zone = Zone.LEFT;
                break;
            case MIDDLE:
                telemetry.addLine("mid");
                telemetry.update();
                zone = Zone.MID;
                break;
            case NONE:
                telemetry.addLine("right");
                telemetry.update();
                zone = Zone.RIGHT;
                break;
            default:
                zone = Zone.NONE;
                break;
        }
    }

    @Override
    public void loop() {


        if(zone==Zone.LEFT) {
            telemetry.addLine("left for real");
            telemetry.update();

        }

        if(zone==Zone.RIGHT) {
            telemetry.addLine("right for real");
            telemetry.update();

        }
        if(zone==Zone.MID) {
            telemetry.addLine("mid af");
            telemetry.update();

        }
        if(zone==Zone.NONE) {
            telemetry.addLine("we messed up big time");
            telemetry.update();

        }

        /*
        double feed = feedforward.calculate(target,2,3);
        double power = feed;

        armMotor1.setPower(power);
        armMotor2.setPower(power);
        */



    }
}
