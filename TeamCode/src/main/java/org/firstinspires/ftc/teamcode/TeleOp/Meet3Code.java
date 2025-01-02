package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
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
    private static final double EXTENDED_SLIDE_TICKS = - 810;
    private static final double ARM_SCORE_POSITION = -100;
    private static final double ARM_COLLECT_POSITION = 1200;

    private static final double SERVO_POWER_INTAKE = 1.0;
    private static final double SERVO_POWER_OUT = -0.5;

    public static boolean manual = false;

    private ColorSensor colorSensor;

    private double blueValue;
    private double greenValue;
    private double redValue;

    private double alphaValue;




    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorV3");
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");
        slideRight = hardwareMap.dcMotor.get("slideR");
        slideLeft = hardwareMap.dcMotor.get("slideL");
        armRotatorRight = hardwareMap.dcMotor.get("armRR");
        armRotatorLeft = hardwareMap.dcMotor.get("armRL");
        intake = hardwareMap.crservo.get("intake");
        wrist = hardwareMap.servo.get("wrist");

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armRotatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armRotatorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        slideRight.setDirection((DcMotorEx.Direction.FORWARD));

        resetMotorEncoders();


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        PIDController armPID = new PIDController(0.01, 0, 0);
        armPID.setTolerance(20,50);





        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            driveControl();
            intakeControl();
            wristControl();
            if (gamepad2.back) {
                manual = true;
            } else if (gamepad2.dpad_up) {
                manual = false;
            }
            if (manual) {
                slideManual();
               // armManual();
            } else {
                slideControl();
                armControl();
            }
            telemetry.addData("SlideL", slideLeft.getCurrentPosition());
            telemetry.addData("SlideR", slideRight.getCurrentPosition());
            telemetry.addData("armL", armRotatorLeft.getCurrentPosition());
            telemetry.addData("armR", armRotatorRight.getCurrentPosition());
            telemetry.addData("wrist", wrist.getPosition());
            telemetry.update();
        }
    }


    private void resetMotorEncoders() {
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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


    private void slideManual() {
        double powerUp = gamepad2.left_stick_y;
        double powerDown = gamepad2.right_stick_y;
        if (Math.abs(powerUp) > 0.1 && slideRight.getCurrentPosition() < EXTENDED_SLIDE_TICKS) {
            slideRight.setPower(((powerUp - powerDown) * 0.3));
            slideLeft.setPower(((powerUp - powerDown) * 0.3));
        }
    }


    private void driveControl() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.right_stick_x;
        double rx = gamepad1.left_stick_x;

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


    private void slideControl() {
        if (gamepad2.a) {
            setSlidePosition((int) EXTENDED_SLIDE_TICKS);
        } else if (gamepad2.b) {
            setSlidePosition(0);
        }
    }


    private void setSlidePosition(int targetPosition) {
        slideRight.setPower(0.5);
        slideRight.setTargetPosition(targetPosition);

        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(0.5);
        slideLeft.setTargetPosition(targetPosition);

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

        private void intakeControl() {
            blueValue = colorSensor.blue();
            greenValue = colorSensor.green();
            redValue = colorSensor.red();
            alphaValue = colorSensor.alpha();
            telemetry.addData("redvalue","%.2f", redValue );
            telemetry.addData("bluevalue","%.2f", blueValue );
            telemetry.addData("greenvalue","%.2f", greenValue );
            telemetry.addData("alphavalue","%.2f", alphaValue );
            if (gamepad1.a) {
                if (blueValue > 2500 && greenValue < 1500 && redValue < 1500) {
                    intake.setPower(0);
                } else if (blueValue < 2500 && greenValue > 2500 && redValue > 2500) {
                    intake.setPower(0);
                } else if (blueValue < 2500 && greenValue < 1500 && redValue > 2500) {
                    intake.setPower(0.4);
                } else {
                    intake.setPower(0.7);
                }
            } else {
                intake.setPower(0);
            }

        }

            private void armControl () {
                if (gamepad2.left_bumper) {
                    runToPosition(ARM_SCORE_POSITION);
                } else if (gamepad2.right_bumper) {
                    runToPosition(ARM_COLLECT_POSITION);
                    manual = true;
                }
            }
            private void armManual () {
                double powerFor = gamepad1.left_trigger;
                double powerRev = gamepad1.right_trigger;
                if (Math.abs(powerFor) > 0.1 && armRotatorLeft.getCurrentPosition() < ARM_SCORE_POSITION && armRotatorRight.getCurrentPosition() < ARM_SCORE_POSITION) {
                    armRotatorLeft.setPower((powerFor - powerRev) * 0.3);
                    armRotatorRight.setPower((powerFor - powerRev) * 0.3);
                }
            }
            private void wristControl () {
                if (gamepad1.right_bumper) {
                    wrist.setPosition(0.6);
                } else if (gamepad1.left_bumper) {
                    wrist.setPosition(0.7);
                }
            }

    }



