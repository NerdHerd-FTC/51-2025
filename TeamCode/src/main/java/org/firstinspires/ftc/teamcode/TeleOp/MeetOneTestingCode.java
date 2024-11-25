package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MeetOneTestingCode")
public class MeetOneTestingCode extends LinearOpMode {
    public CRServo servo;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor slideRight;
    private DcMotor slideLeft;
    private DcMotor armRotatorRight;
    private DcMotor armRotatorLeft;
    private IMU imu;

    // Constants
    private static final double TICKS_PER_DEGREE = 537.7 / 360;
    private static final double EXTENDED_SLIDE_TICKS = TICKS_PER_DEGREE * 720;
    private static final double ARM_SCORE_POSITION = TICKS_PER_DEGREE * 110;
    private static final double ARM_COLLECT_POSITION = 0;
    private static final double SERVO_POWER_INTAKE = 1.0;
    private static final double SERVO_POWER_OUT = -0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        slideRight = hardwareMap.dcMotor.get("slideRight");
        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        armRotatorRight = hardwareMap.dcMotor.get("rightRotate");
        armRotatorLeft = hardwareMap.dcMotor.get("leftRotator");
        servo = hardwareMap.get(CRServo.class, "intake");

        // Motor directions and behaviors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotatorRight.setDirection(DcMotor.Direction.FORWARD);
        armRotatorLeft.setDirection(DcMotor.Direction.FORWARD);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset and set modes
        resetMotorEncoders();

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Main control loop
            driveControl();
            armControl();
            servoControl();
            slideControl();
        }
    }

    private void driveControl() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.start) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;  // Counteract imperfect strafing

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

    private void armControl() {
        if (gamepad2.left_bumper) {
            runToPosition(ARM_SCORE_POSITION);
        } else if (gamepad2.right_bumper) {
            runToPosition(ARM_COLLECT_POSITION);
        }
    }

    private void servoControl() {
        if (gamepad1.a) {
            servo.setPower(SERVO_POWER_INTAKE);
        } else if (gamepad1.b) {
            servo.setPower(SERVO_POWER_OUT);
        } else {
            servo.setPower(0);
        }
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

    private void slideControl() {
        if (gamepad2.a) {
            setSlidePosition((int) EXTENDED_SLIDE_TICKS);
        } else if (gamepad2.b) {
            setSlidePosition(0);
        }
    }

    private void setSlidePosition(int targetPosition) {
        slideLeft.setTargetPosition(targetPosition);
        slideRight.setTargetPosition(targetPosition);
        slideLeft.setPower(0.5);
        slideRight.setPower(0.5);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
}
