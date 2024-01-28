package org.firstinspires.ftc.teamcode;

import static java.lang.Boolean.FALSE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;



@Config
@Autonomous(name="RedLeftAsyncAuto", group="Autonomous")
public class AsyncTest extends OpMode{


    //Arm
    DcMotorEx armMotor1;
    DcMotorEx armMotor2;

    private PIDController controller;
    public static double p = 0.05, i = 0, d = 0;
    public static double f = 3;
    public static int target = 0;

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        armMotor1 = hardwareMap.get(DcMotorEx.class, "arm1");
        armMotor2 = hardwareMap.get(DcMotorEx.class, "arm2");

        Pose2d startPose = new Pose2d(11, -61, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    target = 15;
                })
                .waitSeconds(2)
                .forward(15)
                .waitSeconds(2)
                .back(10)
                .build();

        drive.followTrajectorySequenceAsync(myTrajectory);
    }

    @Override
    public void loop() {
        drive.update();


        controller.setPID(p, i, d);
        int armPos = armMotor1.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ticks_in_degree = 1.6;
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid * ff;
        armMotor1.setPower(power);



        armMotor2.setPower(power);


        telemetry.addData("pos ", armMotor1.getCurrentPosition());
        telemetry.addData("target ", target);
        telemetry.addData("power ", "nopower");

        telemetry.update();
    }
}
