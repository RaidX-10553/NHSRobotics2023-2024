package org.firstinspires.ftc.teamcode.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp
public class PIDF_Controller extends OpMode{
    private PIDController controller;

    public static double p = 0.22, i = 0, d = 0.0002;
    public static double f = 1.1;

    public static int target = 0;

    private DcMotorEx armMotor1;
    private DcMotorEx armMotor2;



    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor1 = hardwareMap.get(DcMotorEx.class, "arm1");
        armMotor2 = hardwareMap.get(DcMotorEx.class, "arm2");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = armMotor1.getCurrentPosition();
        double pid = controller.calculate(armPos, target);

        double ticks_in_degree = 1.6;
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid * ff;

        armMotor1.setPower(power);
        armMotor2.setPower(power);



        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.addData("power ", power);

        telemetry.update();
    }
}