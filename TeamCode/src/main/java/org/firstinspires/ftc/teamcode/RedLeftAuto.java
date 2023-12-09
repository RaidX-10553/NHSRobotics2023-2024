package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="RedLeftAuto", group="Autonomous")
public class RedLeftAuto extends LinearOpMode {

    ServoImplEx claw1;
    ServoImplEx claw2;
    @Override
    public void runOpMode() {

        //Still working on the trajectories, not final
        //Road Runner Trajectory


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        claw1 = hardwareMap.get(ServoImplEx.class, "claw1");
        claw2 = hardwareMap.get(ServoImplEx.class, "claw2");
        claw1.setPwmRange(new PwmControl.PwmRange(500,2500));
        claw2.setPwmRange(new PwmControl.PwmRange(500,2500));


        Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        //flipping
        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    claw1.setPosition(0.25);
                    claw2.setPosition(0.42);
                })
                .waitSeconds(10)
                .strafeRight(4)
                .back (90)
                .build();

        waitForStart();
        drive.followTrajectorySequence(myTrajectory);

        if (isStopRequested()) return;

        while (opModeIsActive()) {


        }
    }
}