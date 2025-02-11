package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "FIELDILT")
@Config
public class BackUpTeleOP extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotorEx slideRight;
    private IMU imu;
    private DcMotorEx slideLeft;
    private DcMotorEx armRotatorRight;
    private DcMotorEx armRotatorLeft;

    private CRServo rollerR;

    private CRServo accR;

    private CRServo accL;



    private Servo inWrist;

    private Servo grabR;
    private Servo grabL;

    private Servo armR;
    private Servo armL;

    private Servo claw;
    private Servo roClaw;




    private static final double TICKS_PER_DEGREE = 537.7 / 360;
    private static final double EXTENDED_SLIDE_TICKS = 2450;

    private static final double COLLECT_SLIDE_TICKS = 810;

    private static final double ARM_SCORE_POSITION = -500;
    private static final double ARM_COLLECT_POSITION = 985;

    private static final double SERVO_POWER_INTAKE = 1.0;
    private static final double SERVO_POWER_OUT = -0.5;

    public static boolean manual = false;

    public int targetPosition;


    private PIDController controller;
    public static double p = 0.00, i = 0.000, d = 0.000;
    public static double f = 0.000;


    public static int target = 0;

    private final double tick_per_degree = 537.7 / 360;


    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");
        slideRight = (DcMotorEx) hardwareMap.dcMotor.get("slideR");
        slideLeft = (DcMotorEx) hardwareMap.dcMotor.get("slideL");
        armRotatorRight = (DcMotorEx) hardwareMap.dcMotor.get("armRR");
        armRotatorLeft = (DcMotorEx) hardwareMap.dcMotor.get("armRL");
        rollerR = hardwareMap.crservo.get("rollerR");
        accR = hardwareMap.crservo.get("accR");
        accL = hardwareMap.crservo.get("accL");


        inWrist = hardwareMap.servo.get("inWrist");
        claw = hardwareMap.servo.get("claw");
        roClaw = hardwareMap.servo.get("roClaw");




        grabR = hardwareMap.servo.get("grabR");
        grabL = hardwareMap.servo.get("grabL");

        armR = hardwareMap.servo.get("armR");
        armL = hardwareMap.servo.get("armL");


        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armRotatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armRotatorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setDirection(DcMotorEx.Direction.FORWARD);
        slideRight.setDirection((DcMotorEx.Direction.REVERSE));

        armRotatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rollerR.setDirection(CRServo.Direction.REVERSE);


        resetMotorEncoders();
        int armRPos = armRotatorRight.getCurrentPosition();
        int armLPos = armRotatorLeft.getCurrentPosition();
        telemetry.addData("posR", armRPos);
        telemetry.addData("posL", armLPos);


        telemetry.update();


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);


        controller = new PIDController(p, i, d);


        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            driveControl();
            //   intakeControl();
            wristControl();


//            if (gamepad2.back) {
//                manual = true;
//            } else if (gamepad2.dpad_up) {
//                manual = false;
//            }


            // armManual();

            // armControl();
            armPIDControl();
            intakeControl();
            grabControl();
            clawControl();
            roClawControl();

            actControl();

            telemetry.addData("SlideL", slideLeft.getCurrentPosition());
            telemetry.addData("SlideR", slideRight.getCurrentPosition());
            telemetry.addData("armL", armRotatorLeft.getCurrentPosition());
            telemetry.addData("armR", armRotatorRight.getCurrentPosition());
            double wristPos = inWrist.getPosition();

            telemetry.addData("wristPos", wristPos);

        }
    }


    private void resetMotorEncoders() {
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    private void driveControl() {
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double x = gamepad1.left_stick_x;

        if (gamepad1.start) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(-backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);


    }


//    private void runToPosition(double targetPosition) {
//        int newTarget = (int) (TICKS_PER_DEGREE * targetPosition);
//        armRotatorLeft.setTargetPosition(newTarget);
//        armRotatorLeft.setPower(0.5);
//        armRotatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        armRotatorRight.setTargetPosition(newTarget);
//        armRotatorRight.setPower(0.5);
//        armRotatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }

    private void intakeControl() {

        if (gamepad1.a) {
            rollerR.setPower(.7);
        } else if (gamepad1.b) {
            rollerR.setPower(-.3);
        } else {
            rollerR.setPower(0);
        }


    }
    private void actControl () {
        if (gamepad1.x) {
            accR.setPower(1);
            accL.setPower(1);
        } else if (gamepad1.y) {
            accR.setPower(-1);
            accL.setPower(-1);
        }
    }


    private void armPIDControl() {
        int armRPos = armRotatorRight.getCurrentPosition();
        int armLPos = armRotatorLeft.getCurrentPosition();
        int slideRPos = armRotatorRight.getCurrentPosition();
        int slideLPos = armRotatorLeft.getCurrentPosition();

        int currentP = (int) ((armRPos + armLPos + slideRPos + slideLPos) / 4);
        double pid = controller.calculate(currentP, target);

        double ff = Math.cos(Math.toRadians(target / tick_per_degree)) * f;

        double power = pid + ff;
        if (gamepad2.left_bumper) {
            target = -500;
            manual = false;
        } else if (gamepad2.right_bumper) {
            target = 975;
            manual = true;
        } else if (gamepad2.dpad_right && armRPos < 1050 && armLPos < 1050) {
            target = target + 25;
        } else if (gamepad2.dpad_left) {
            target = target - 25;
        }


        armRotatorLeft.setPower(power);
        armRotatorRight.setPower(power);
        slideRight.setPower(power);
        slideLeft.setPower(power);

        telemetry.addData("posR", armRPos);
        telemetry.addData("posL", armLPos);
        telemetry.addData("target", target);
        telemetry.update();
    }

    private void wristControl() {
        if (gamepad1.right_bumper) {
            inWrist.setPosition(0.7);
        } else if (gamepad1.left_bumper) {
            inWrist.setPosition(0.175);
        }
    }

    private void grabControl() {
        if (gamepad2.a) {
            grabR.setPosition(.7);
            grabL.setPosition(.7);
        } else if (gamepad2.b) {
            grabR.setPosition(0.0);
            grabL.setPosition(0.0);
        } else if (gamepad2.x && grabR.getPosition() < 0.7) {
            grabR.setPosition(grabR.getPosition() + 0.2);
            grabL.setPosition(grabR.getPosition() + 0.2);
        } else if (gamepad2.y && grabR.getPosition() > 0.0) {
            grabR.setPosition(grabR.getPosition() - 0.2);
            grabL.setPosition(grabR.getPosition() - 0.2);
        }
    }

    private void armControl() {
        if (gamepad2.left_bumper) {
            armR.setPosition(.7);
            armL.setPosition(.7);
        } else if (gamepad2.right_bumper) {
            armR.setPosition(0.2);
            armL.setPosition(0.2);
        }
    }

    private void clawControl() {
        if (gamepad1.right_bumper) {
            claw.setPosition(.7);
        } else if (gamepad1.left_bumper) {
            claw.setPosition(0.2);
        }
    }

    private void roClawControl() {
        if (gamepad1.dpad_left) {
            roClaw.setPosition(.7);
        } else if (gamepad1.dpad_right) {
            roClaw.setPosition(0.2);
        }
    }





}




