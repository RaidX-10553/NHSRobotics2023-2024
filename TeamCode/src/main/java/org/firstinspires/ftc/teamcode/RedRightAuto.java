package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="RedRightAuto", group="Autonomous")
public class RedRightAuto extends LinearOpMode {


    @Override
    public void runOpMode() {

        //Still working on the trajectories, not final
        //Road Runner Trajectory


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(11, -61, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        //flipping
        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .strafeRight(2)
                .waitSeconds(1)
                .back(75)
                .waitSeconds(1)
                .strafeRight(47)
                .waitSeconds(1)
                .back(20)
                .build();

        while (opModeIsActive()) {


        }
    }
}