package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystem.ColorDetector.centerRect;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ColorDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;


@Autonomous(name="BlueRightAuto", group="Autonomous")
public class BlueRightAuto extends LinearOpMode {

    ColorDetector detector;

    VisionPortal visionPortal;


    public int zone = 0;

    ServoImplEx claw1;
    ServoImplEx claw2;


    @Override
    public void runOpMode() {
        Rect leftZone = centerRect(240,240,150,300);
        Rect midZone = centerRect(560,180,150,200);

        detector = new ColorDetector(telemetry, ColorDetector.TargetColor.BLUE, ColorDetector.ViewMode.RAW, leftZone, midZone);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), detector);
        CameraStreamServer.getInstance().setSource(detector);

        telemetry.addData("Detection captured:",detector.getConfidentDetection());
        ColorDetector.Detection detection = detector.getConfidentDetection();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Claw
        claw1 = hardwareMap.get(ServoImplEx.class, "claw1");
        claw2 = hardwareMap.get(ServoImplEx.class, "claw2");
        claw1.setPwmRange(new PwmControl.PwmRange(500,2500));
        claw2.setPwmRange(new PwmControl.PwmRange(500,2500));

        Pose2d startPose = new Pose2d(-40, -61, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(.40);
                    claw2.setPosition(0.30);
                    telemetry.addData("Closing ","");
                    telemetry.update();
                })
                .waitSeconds(1)
                .strafeRight(4)
                .forward(26)
                .turn(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(0.50);
                    claw2.setPosition(0.19);
                    telemetry.addData("Opening ","");
                    telemetry.update();
                })
                .strafeLeft(8)
                .back(10)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(.40);
                    claw2.setPosition(0.30);
                    telemetry.addData("Closing ","");
                    telemetry.update();
                })
                .waitSeconds(1)
                .strafeRight(4)
                .forward(31)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(0.50);
                    claw2.setPosition(0.19);
                    telemetry.addData("Opening ","");
                    telemetry.update();
                })
                .waitSeconds(2)
                .back(10)
                .build();


        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(.40);
                    claw2.setPosition(0.30);
                    telemetry.addData("Closing ","");
                    telemetry.update();
                })
                .waitSeconds(1)
                .strafeRight(4)
                .forward(26)
                .turn(Math.toRadians(90))
                .strafeRight(8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(0.50);
                    claw2.setPosition(0.19);
                    telemetry.addData("Opening ","");
                    telemetry.update();
                })
                .back(10)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        if(detector.getConfidentDetection() == ColorDetector.Detection.LEFT) {
            telemetry.addLine("left detected");
            zone = 1;
            telemetry.update();

        }

        if(detector.getConfidentDetection() == ColorDetector.Detection.MIDDLE) {
            telemetry.addLine("mid detected");
            zone = 3;
            telemetry.update();

        }


        if(zone == 1){
            telemetry.addLine("going left");
            telemetry.update();

            drive.followTrajectorySequence(left);

        }

        if(zone == 3){
            telemetry.addLine("going mid");
            telemetry.update();

            drive.followTrajectorySequence(middle);
        }

        if(zone == 0){
            telemetry.addLine("going right");
            telemetry.update();

            drive.followTrajectorySequence(right);
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (opModeIsActive()) {

        }
    }
}