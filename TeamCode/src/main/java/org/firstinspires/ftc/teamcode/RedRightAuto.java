package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystem.ColorDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;
import  static org.firstinspires.ftc.teamcode.subsystem.ColorDetector.*;



@Config
@Autonomous(name="RedRightAuto", group="Autonomous")
public class RedRightAuto extends OpMode{
    private ArmFeedforward feedforward;

    public static double kS = 0, kV = 0, kA = 0;
    public static double kCos = 0;

    public static int target = 0;

    private DcMotorEx armMotor1;
    private DcMotorEx armMotor2;

    ColorDetector detector;

    VisionPortal visionPortal;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


    @Override
    public void init() {
        feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Rect leftZone = centerRect(120,280,150,200);
        Rect midZone = centerRect(320,320,150,200);

        detector = new ColorDetector(telemetry, TargetColor.RED, ColorDetector.ViewMode.RAW, leftZone, midZone);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), detector);
        CameraStreamServer.getInstance().setSource(detector);

        telemetry.addData("Detection captured:",detector.getConfidentDetection());
        Detection detection = detector.getConfidentDetection();

        armMotor1 = hardwareMap.get(DcMotorEx.class, "arm1");
        armMotor2 = hardwareMap.get(DcMotorEx.class, "arm2");

        Pose2d startPose = new Pose2d(11, -61, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                })
                .strafeRight(2)
                .back(50)
                .build();

        drive.followTrajectorySequenceAsync(myTrajectory);

    }

    @Override
    public void loop() {
        drive.update();


        /*
        double feed = feedforward.calculate(target,2,3);
        double power = feed;

        armMotor1.setPower(power);
        armMotor2.setPower(power);
        */

        telemetry.addData("pos ", armMotor1.getCurrentPosition());
        telemetry.addData("target ", target);
        telemetry.addData("power ", "nopower");

        telemetry.update();
    }
}