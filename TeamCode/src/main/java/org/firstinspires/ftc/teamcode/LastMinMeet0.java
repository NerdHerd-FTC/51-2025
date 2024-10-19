package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



    @TeleOp(name = "ImprovisedMeet0")
    public class LastMinMeet0 extends LinearOpMode {
        //public CRServo servo;
        @Override
        public void runOpMode() throws InterruptedException {
            // Declare our motors
            // Make sure your ID's match your configuration
            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
            DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
          // servo = hardwareMap.get(CRServo.class, "intake");
            // Reverse the right side motors. This may be wrong for your setup.
            // If your robot moves backwards when commanded to go forwards,
            // reverse the left side instead.
            // See the note about this earlier on this page.
            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;
                double armPower = gamepad2.left_stick_y*0.15 ;

               /* if (gamepad1.a) {
                    servo.setPower(1);
                } else if (gamepad1.b) {
                    servo.setPower(-0.5);
                } else {
                    servo.setPower(0);
                }*/

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = 0.8*(y + x + rx) / denominator;
                double backLeftPower = 0.8*-(y - x + rx) / denominator;
                double frontRightPower = 0.8*-(y - x - rx) / denominator;
                double backRightPower = 0.8*-(y + x - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
                armMotor.setPower(armPower);
            }
        }
    }



