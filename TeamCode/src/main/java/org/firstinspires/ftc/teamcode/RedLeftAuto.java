package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystem.ColorDetector.Detection;
import static org.firstinspires.ftc.teamcode.subsystem.ColorDetector.TargetColor;
import static org.firstinspires.ftc.teamcode.subsystem.ColorDetector.ViewMode;
import static org.firstinspires.ftc.teamcode.subsystem.ColorDetector.centerRect;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.subsystem.ColorDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Config
@Autonomous(name="RedLeftAuto", group="Autonomous")
public class RedLeftAuto extends OpMode{

    ColorDetector detector;

    Trajectory trajectoryLeft;
    VisionPortal visionPortal;


    public int zone = 0;

    ServoImplEx claw1;
    ServoImplEx claw2;



    @Override
    public void init() {

        //y is the opposite of what you think
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Claw
        claw1 = hardwareMap.get(ServoImplEx.class, "claw1");
        claw2 = hardwareMap.get(ServoImplEx.class, "claw2");
        claw1.setPwmRange(new PwmControl.PwmRange(500,2500));
        claw2.setPwmRange(new PwmControl.PwmRange(500,2500));


        //Pose2d startPose = new Pose2d(35.25, 64, Math.toRadians(270));


        //actually left???
        TrajectorySequence right = drive.trajectorySequenceBuilder( new Pose2d(-38, -61, Math.toRadians(90)))
                .strafeLeft(20)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(.40);
                    claw2.setPosition(0.30);
                    telemetry.addData("Opening ","");
                    telemetry.update();
                })
                .waitSeconds(1)
                .strafeLeft(19)
                .forward(25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(0.50);
                    claw2.setPosition(0.19);
                    telemetry.addData("Opening ","");
                    telemetry.update();
                })
                .back(2)
                //.strafeRight(39)
                //.forward(80)
                .waitSeconds(20)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder( new Pose2d(-38, -61, Math.toRadians(90)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(.40);
                    claw2.setPosition(0.30);
                    telemetry.addData("Opening ","");
                    telemetry.update();
                })
                .waitSeconds(1)
                .strafeLeft(8)
                .forward(34)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(0.50);
                    claw2.setPosition(0.19);
                    telemetry.addData("Opening ","");
                    telemetry.update();
                })
                .back(2)
                //.strafeRight(39)
                //.forward(80)
                .waitSeconds(20)
                .build();

        //actually right???
        TrajectorySequence left = drive.trajectorySequenceBuilder( new Pose2d(-38, -61, Math.toRadians(90)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(.40);
                    claw2.setPosition(0.30);
                    telemetry.addData("Opening ","");
                    telemetry.update();
                })
                .waitSeconds(1)
                .forward(31)
                .strafeRight(19)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //Claw Opens
                    claw1.setPosition(0.50);
                    claw2.setPosition(0.19);
                    telemetry.addData("Opening ","");
                    telemetry.update();
                })
                .back(1)
                .waitSeconds(20)
                .build();


        if(zone == 1){
            telemetry.addLine("going left");
            telemetry.update();

            drive.followTrajectorySequence(left);

        }
        if(zone == 0){
            telemetry.addLine("going right");
            telemetry.update();

            drive.followTrajectorySequence(right);



        }
        if(zone == 3){
            telemetry.addLine("going mid");
            telemetry.update();


            drive.followTrajectorySequence(middle);
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
