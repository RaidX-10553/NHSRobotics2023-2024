package org.firstinspires.ftc.teamcode.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp
public class PIDF_Controller extends OpMode{
    private ArmFeedforward feedforward;

    public static double kS = 0, kV = 0, kA = 0;
    public static double kCos = 0;

    public static int target = 0;

    private DcMotorEx armMotor1;
    private DcMotorEx armMotor2;



    @Override
    public void init() {
        feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor1 = hardwareMap.get(DcMotorEx.class, "arm1");
        armMotor2 = hardwareMap.get(DcMotorEx.class, "arm2");


    }

    @Override
    public void loop() {
        double feed = feedforward.calculate(target,2,3);


        double power = feed;

        armMotor1.setPower(power);
        armMotor2.setPower(power);



        telemetry.addData("pos ", armMotor1.getCurrentPosition());
        telemetry.addData("target ", target);
        telemetry.addData("power ", power);

        telemetry.update();
    }
}