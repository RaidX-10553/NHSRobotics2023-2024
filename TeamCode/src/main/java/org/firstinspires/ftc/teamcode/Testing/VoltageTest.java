package org.firstinspires.ftc.teamY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * test for amperage monitoring of a motor
 * graphs the amperage of the motor on the dashboard
 */
@Config
@Autonomous
public class VoltageTest extends LinearOpMode {
    DcMotorEx Arm;



    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        Arm = hardwareMap.get(DcMotorEx.class, "arm");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.a){
                Arm.setPower(1);
            }

            if(gamepad1.b){
                Arm.setPower(0);
            }

            if(gamepad1.x){
                double power = gamepad1.left_stick_y;
                Arm.setPower(power);
            }


            telemetry.addData("amps", Arm.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            sleep(20);
            
        }
    }
}
