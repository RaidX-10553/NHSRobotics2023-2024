package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ColorDetector.Detection;
import static org.firstinspires.ftc.teamcode.ColorDetector.TargetColor;
import static org.firstinspires.ftc.teamcode.ColorDetector.ViewMode;
import static org.firstinspires.ftc.teamcode.ColorDetector.centerRect;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;


@Config
@Autonomous(name="VisionTest", group="Autonomous")
public class VisionTest extends OpMode{

    ColorDetector detector;

    VisionPortal visionPortal;



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
        switch (detector.getConfidentDetection()) {
            case LEFT:
                telemetry.addLine("left");
                telemetry.update();
                break;
            case MIDDLE:
                telemetry.addLine("mid");
                telemetry.update();
                break;
            case NONE:
                telemetry.addLine("right");
                telemetry.update();
                break;
            default:
                //park
                break;
        }


        /*
        double feed = feedforward.calculate(target,2,3);
        double power = feed;

        armMotor1.setPower(power);
        armMotor2.setPower(power);
        */



    }
}
