package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "MeetTwoTestingCode")
@Config
public class Meet3Code extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor slideRight;
    private IMU imu;
    private DcMotor slideLeft;
    private DcMotor armRotatorRight;
    private DcMotor armRotatorLeft;

    private CRServo intake;

    private Servo wrist;


    private static final double TICKS_PER_DEGREE = 537.7 / 360;
    private static final double EXTENDED_SLIDE_TICKS = TICKS_PER_DEGREE * 720;
    private static final double ARM_SCORE_POSITION = TICKS_PER_DEGREE * 110;
    private static final double ARM_COLLECT_POSITION = 0;

    private static final double SERVO_POWER_INTAKE = 1.0;
    private static final double SERVO_POWER_OUT = -0.5;

    public static boolean manual = false;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        slideRight = hardwareMap.dcMotor.get("slideR");
        slideLeft = hardwareMap.dcMotor.get("slideL");
        armRotatorRight = hardwareMap.dcMotor.get("armRR");
        armRotatorLeft = hardwareMap.dcMotor.get("armRL");
//        intake = (CRServo) hardwareMap.servo.get("intake");
//        wrist =  hardwareMap.servo.get("wrist");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        resetMotorEncoders();


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);



        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            driveControl();
           // intakeControl();
          //  wristControl();
            if (gamepad2.start) {
                manual = true;
            } else if (gamepad2.back) {
                manual = false;
            }
            if (manual) {
                slideManual();
                armManual();
            } else {
                slideControl();
                armControl();
            }
        }
    }


    private void resetMotorEncoders() {
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    private void slideManual() {
        double powerUp = gamepad2.left_stick_y;
        double powerDown = gamepad2.right_stick_y;
        if (Math.abs(powerUp) > 0.1 && slideRight.getCurrentPosition() < EXTENDED_SLIDE_TICKS) {
            slideRight.setPower((powerUp - powerDown) * 0.3);
        }
    }


    private void driveControl() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;


        if (gamepad1.start) {
            imu.resetYaw();
        }


        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }


    private void slideControl() {
        if (gamepad2.a) {
            setSlidePosition((int) EXTENDED_SLIDE_TICKS);
        } else if (gamepad2.b) {
            setSlidePosition(0);
        }
    }


    private void setSlidePosition(int targetPosition) {
        slideRight.setTargetPosition(targetPosition);
        slideRight.setPower(0.5);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setTargetPosition(targetPosition);
        slideLeft.setPower(0.5);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void runToPosition(double targetPosition) {
        int newTarget = (int) (TICKS_PER_DEGREE * targetPosition);
        armRotatorLeft.setTargetPosition(newTarget);
        armRotatorLeft.setPower(0.5);
        armRotatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armRotatorRight.setTargetPosition(newTarget);
        armRotatorRight.setPower(0.5);
        armRotatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

//    private void intakeControl() {
//        if (gamepad1.a) {
//            intake.setPower(SERVO_POWER_INTAKE);
//        } else if (gamepad1.b) {
//            intake.setPower(SERVO_POWER_OUT);
//        } else {
//            intake.setPower(0);
//        }
//    }

    private void armControl() {
        if (gamepad2.left_bumper) {
            runToPosition(ARM_SCORE_POSITION);
        } else if (gamepad2.right_bumper) {
            runToPosition(ARM_COLLECT_POSITION);
        }
    }
    private void armManual(){
        double powerFor = gamepad1.left_trigger;
        double powerRev = gamepad1.right_trigger;
        if (Math.abs(powerFor) > 0.1 && armRotatorLeft.getCurrentPosition() < ARM_SCORE_POSITION && armRotatorRight.getCurrentPosition() < ARM_SCORE_POSITION) {
            armRotatorLeft.setPower((powerFor - powerRev) * 0.3);
            armRotatorRight.setPower((powerFor - powerRev) * 0.3);
        }
    }
//    private void wristControl() {
//        if(gamepad1.right_bumper) {
//            wrist.setPosition(1);
//        } else if (gamepad1.left_bumper) {
//            wrist.setPosition(0);
//        }
//    }
}


