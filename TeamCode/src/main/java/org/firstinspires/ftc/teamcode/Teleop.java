package org.firstinspires.ftc.teamcode;



import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@TeleOp(name = "Teleop ArmV1", group = "TeleOp")
public class Teleop extends LinearOpMode {
    //Claw
    ServoImplEx claw1;
    ServoImplEx claw2;

    //Drive
    SampleMecanumDrive mecanumDrive;

    //Arm
    DcMotorEx armMotor1;
    DcMotorEx armMotor2;

    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    private PIDController controller;
    public static double p = 0.05, i = 0, d = 0;
    public static double f = 3;
    public static int target = 0;
    private double driveValue = 1;
    boolean arm = TRUE;
    @Override
    public void runOpMode() throws InterruptedException {
        //Drive
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        //Claw
        claw1 = hardwareMap.get(ServoImplEx.class, "claw1");
        claw2 = hardwareMap.get(ServoImplEx.class, "claw2");
        claw1.setPwmRange(new PwmControl.PwmRange(500,2500));
        claw2.setPwmRange(new PwmControl.PwmRange(500,2500));

        //Arm
        armMotor1 = hardwareMap.get(DcMotorEx.class, "arm1");
        armMotor2 = hardwareMap.get(DcMotorEx.class, "arm2");

        //What's cracking guys - Syed from FTC
        //Motor Behavior
        controller = new PIDController(p, i, d);




        waitForStart();


        //Code is looped inside this while loop
        while (opModeIsActive()) {

            //try catch
            try{
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);

                throw new RobotCoreException("kill yourself");
            }
            catch (RobotCoreException f)
            {
                //suck me
            }

            //Slow Driving
            if (gamepad1.x) {
                telemetry.addData("Mode", "PRECISE");
                telemetry.update();
                driveValue = 0.4;
            }

            //Reversed Driving
            if (gamepad1.b) {
                telemetry.addData("Mode", "REVERSED");
                telemetry.update();
                driveValue = -1;
            }

            //Normal Driving
            if (gamepad1.a) {
                telemetry.addData("Mode", "Normal");
                telemetry.update();
                driveValue = 1;
            }


            //Drive Code Using RR
            mecanumDrive.setDrivePower(
                    new Pose2d(-gamepad1.left_stick_y * driveValue,
                            -gamepad1.right_stick_x * driveValue,
                            -gamepad1.left_stick_x * driveValue));
            mecanumDrive.updatePoseEstimate();





            //Arm Control


            if (arm) {
                controller.setPID(p, i, d);
                int armPos = armMotor1.getCurrentPosition();

                double pid = controller.calculate(armPos, target);
                double ticks_in_degree = 1.6;
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
                double power = pid * ff;
                armMotor1.setPower(power);
                armMotor2.setPower(power);
            }





            if (gamepad2.y) {
                arm = TRUE;
                target = 50;
            }

            if (gamepad2.b) {
                arm = TRUE;
                target = 15;
            }

            if (gamepad2.a) {
                arm = FALSE;
                controller.reset();
            }

            if (!arm) {
                armMotor1.setPower(0);
                armMotor2.setPower(0);
                armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }



            if (gamepad2.dpad_down && !arm) {
                double x = -gamepad2.left_stick_y;
                armMotor1.setPower(x*0.6);
                armMotor2.setPower(x*0.6);
            }
            



            //Range is between [0.0 and 1.0] 0.5 being the center
            //Fix position values with testing
            //Claw Toggle
            //Falling Edge Detector
            if (!currentGamepad2.right_bumper && previousGamepad2.right_bumper) {
                if (claw1.getPosition() == 0.50) {
                    claw1.setPosition(0.25);
                } else {
                    claw1.setPosition(0.50);
                }
            }

            if (!currentGamepad2.right_bumper && previousGamepad2.right_bumper) {
                if ((claw2.getPosition()> 0.14)&&(claw2.getPosition() < 0.24)) {
                    claw2.setPosition(0.42);
                } else {
                    claw2.setPosition(0.19);
                }
            }

            telemetry.addData("Arm1", armMotor1.getCurrentPosition());
            telemetry.addData( "Arm2" , armMotor2.getCurrentPosition());
            telemetry.addData("ClawLeft", claw1.getPosition());
            telemetry.addData( "ClawRight" , claw2.getPosition());

            telemetry.update();




        }

    }

}

