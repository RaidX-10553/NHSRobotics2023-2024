package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;



@Autonomous(name="BlueLeftAuto", group="Autonomous")
public class BlueLeftAuto extends LinearOpMode {

    DcMotorEx armMotor1;
    DcMotorEx armMotor2;

    private PIDController controller;
    public static double p = 0.05, i = 0, d = 0;
    public static double f = 3;
    public static int target = 0;

    @Override
    public void runOpMode() {

        //Still working on the trajectories, not final
        //Road Runner Trajectory


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(11, 61, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        armMotor1 = hardwareMap.get(DcMotorEx.class, "arm1");
        armMotor2 = hardwareMap.get(DcMotorEx.class, "arm2");

        //What's cracking guys - Syed from FTC
        //Motor Behavior
        controller = new PIDController(p, i, d);

        //flipping
        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    target = 15;
                })
                .waitSeconds(1)
                .strafeLeft(2)
                .waitSeconds(1)
                .back(29)
                .waitSeconds(1)
                .strafeLeft(47)
                .waitSeconds(1)
                .back(20)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(myTrajectory);

        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int armPos = armMotor1.getCurrentPosition();

            double pid = controller.calculate(armPos, target);
            double ticks_in_degree = 1.6;
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid * ff;
            armMotor1.setPower(power);
            armMotor2.setPower(power);

        }
    }
}