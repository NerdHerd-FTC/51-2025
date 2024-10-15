/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "aaditesting")
public class  Meet0Code extends LinearOpMode {
    public CRServo servo;;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
        servo = hardwareMap.get(CRServo.class, "intake");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        final double ARM_TICKS_PER_DEGREE =
                28
                        * 250047.0 / 4913.0
                        * 100.0 / 20.0
                        * 1/360.0;

        final double ARM_COLLAPSED_INTO_ROBOT  = armMotor.getCurrentPosition();
        final double ARM_SCORE_LOW_BASKET = 180*ARM_TICKS_PER_DEGREE;
        final double ARM_COLLECT= 250 * ARM_TICKS_PER_DEGREE;
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
        double armPositionFudgeFactor;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double armPower = gamepad2.left_stick_y *0.2;

            while(gamepad1.a) {
                servo.setPower(1);
            }
            while(gamepad1.b){
                servo.setPower(-0.5);
            }
            servo.setPower(0);



            if(gamepad2.right_bumper) {
                armPosition =  ARM_SCORE_LOW_BASKET;

            }
            if(gamepad1.left_bumper) {

                armPosition = ARM_COLLECT;


            }




            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = -(y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            armMotor.setPower(armPower);


        }
    }
}*/

/* package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "aaditesting")
public class Meet0Code extends LinearOpMode {
   public CRServo servo;
    final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 / 360.0;
    final double ARM_SCORE_LOW_BASKET = 180 * ARM_TICKS_PER_DEGREE;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 15  * ARM_TICKS_PER_DEGREE;


    double armPosition = (int) ARM_WINCH_ROBOT;
    double armPositionFudgeFactor;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
       servo = hardwareMap.get(CRServo.class, "intake");

        // Reverse the right side motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */

/*import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double armPower = gamepad2.left_stick_y * 0.2;

            if (gamepad1.a) {
                servo.setPower(1);
            } else if (gamepad1.b) {
                servo.setPower(-0.5);
            } else {
                servo.setPower(0);
            }
            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));
            if (gamepad2.right_bumper) {
                armPosition = ARM_SCORE_LOW_BASKET;
            } else if (gamepad1.left_bumper) {
                armPosition = ARM_COLLECT;
            }
            armMotor.setTargetPosition((int)(armPosition+armPositionFudgeFactor));

            // Denominator is th-+largest motor power (absolute value) or 1
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = -(y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            armMotor.setTargetPosition((int) armPosition);
        }
    }

} */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "aaditesting")
public class Meet0Code extends LinearOpMode {
    public CRServo servo;

    final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 / 360.0;
    final double ARM_SCORE_LOW_BASKET = 180 * ARM_TICKS_PER_DEGREE;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;
    double armPosition = ARM_WINCH_ROBOT;
    double armPositionFudgeFactor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
        servo = hardwareMap.get(CRServo.class, "intake");

        // Reverse the right side motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double armPower = gamepad2.left_stick_y * 0.2;

            if (gamepad1.a) {
                servo.setPower(1);
            } else if (gamepad1.b) {
                servo.setPower(-0.5);
            } else {
                servo.setPower(0);
            }

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));
            if (gamepad2.right_bumper) {
                armPosition = ARM_SCORE_LOW_BASKET;
            } else if (gamepad1.left_bumper) {
                armPosition = ARM_COLLECT;
            }

            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

            // Denominator is the largest motor power (absolute value) or 1
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = -(y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

        }
    }
    }