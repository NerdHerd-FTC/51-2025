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
    private DcMotorEx armRotatorRight;
    private DcMotorEx armRotatorLeft;

    //   private CRServo intake;

    //  private Servo wrist;


    private static final double TICKS_PER_DEGREE = 537.7 / 360;
    private static final double EXTENDED_SLIDE_TICKS =  2000;

    private static final double COLLECT_SLIDE_TICKS =  810;

    private static final double ARM_SCORE_POSITION = -500;
    private static final double ARM_COLLECT_POSITION = 1000;

    private static final double SERVO_POWER_INTAKE = 1.0;
    private static final double SERVO_POWER_OUT = -0.5;

    public static boolean manual = false;

    public int targetPosition;


    private ColorSensor colorSensor;

    private double blueValue;
    private double greenValue;
    private double redValue;

    private double alphaValue;

    private PIDController controller;
    public static double p = 0.005, i = 0.0008, d = 0.0002;
    public static double f = 0.0001;

    public static int target = 0;

    private final double tick_per_degree = 537.7/360;






    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorV3");
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");
        slideRight = hardwareMap.dcMotor.get("slideR");
        slideLeft = hardwareMap.dcMotor.get("slideL");
        armRotatorRight = (DcMotorEx) hardwareMap.dcMotor.get("armRR");
        armRotatorLeft = (DcMotorEx) hardwareMap.dcMotor.get("armRL");
        //  intake = hardwareMap.crservo.get("intake");
        //   wrist = hardwareMap.servo.get("wrist");

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
        resetMotorEncoders();


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
            //  intakeControl();
            //  wristControl();
//            if (gamepad2.back) {
//                manual = true;
//            } else if (gamepad2.dpad_up) {
//                manual = false;
//            }

            slideManual();
            // armManual();

            // armControl();
            armPIDControl();

            telemetry.addData("SlideL", slideLeft.getCurrentPosition());
            telemetry.addData("SlideR", slideRight.getCurrentPosition());
            telemetry.addData("armL", armRotatorLeft.getCurrentPosition());
            telemetry.addData("armR", armRotatorRight.getCurrentPosition());
            // telemetry.addData("wrist", wrist.getPosition());
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
        if(slideLeft.getCurrentPosition() < 810 && slideRight.getCurrentPosition() <810 && slideLeft.getCurrentPosition() > -5 && slideRight.getCurrentPosition() > -5 ){
            if(gamepad2.a) {
                targetPosition = targetPosition + 30;

            } else if (gamepad2.b) {
                targetPosition = targetPosition - 30;
            } else {
                targetPosition = (slideRight.getCurrentPosition()+slideLeft.getCurrentPosition())/2 ;
            }
        } else if(slideLeft.getCurrentPosition() < 810 && slideRight.getCurrentPosition() <810 && !(slideLeft.getCurrentPosition() > -5 && slideRight.getCurrentPosition() > -5)) {
            if (gamepad2.b) {
                targetPosition = targetPosition - 30;
            } else {
                targetPosition = (slideRight.getCurrentPosition()+slideLeft.getCurrentPosition())/2 ;
            }
            if (gamepad2.a) {
                targetPosition = targetPosition + 30;
            } else {
                targetPosition = (slideRight.getCurrentPosition() + slideLeft.getCurrentPosition()) / 2;
            }
        } else if(!(slideLeft.getCurrentPosition() < 810 && slideRight.getCurrentPosition() <810) && slideLeft.getCurrentPosition() > -5 && slideRight.getCurrentPosition() > -5) {
            if (gamepad2.b) {
                targetPosition = targetPosition - 30;
            } else {
                targetPosition = (slideRight.getCurrentPosition()+slideLeft.getCurrentPosition())/2 ;
            }
        }   else {
            targetPosition = (slideRight.getCurrentPosition() + slideLeft.getCurrentPosition()) / 2;
        }
        if (gamepad2.x) {
            targetPosition = (int) EXTENDED_SLIDE_TICKS;

        } else if (gamepad2.y) {
            targetPosition = 10;
        }

        slideRight.setPower(0.5);
        slideRight.setTargetPosition(targetPosition);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideLeft.setPower(0.5);
        slideLeft.setTargetPosition(targetPosition);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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


    private void slideControl() {
        if (gamepad2.a) {
            targetPosition = (int) EXTENDED_SLIDE_TICKS;

        } else if (gamepad2.b) {
            targetPosition = 10;
        }
    }


    private void setSlidePosition() {
        slideRight.setPower(0.5);
        slideRight.setTargetPosition(targetPosition);

        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(0.5);
        slideLeft.setTargetPosition(targetPosition);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

//        private void intakeControl() {
//            blueValue = colorSensor.blue();
//            greenValue = colorSensor.green();
//            redValue = colorSensor.red();
//            alphaValue = colorSensor.alpha();
//            telemetry.addData("redvalue","%.2f", redValue );
//            telemetry.addData("bluevalue","%.2f", blueValue );
//            telemetry.addData("greenvalue","%.2f", greenValue );
//            telemetry.addData("alphavalue","%.2f", alphaValue );
//            if (gamepad1.a) {
//                if (blueValue > 2500 && greenValue < 1500 && redValue < 1500) {
//                    intake.setPower(0);
//                } else if (blueValue < 2500 && greenValue > 2500 && redValue > 2500) {
//                    intake.setPower(0);
//                } else if (blueValue < 2500 && greenValue < 1500 && redValue > 2500) {
//                    intake.setPower(0.4);
//                } else {
//                    intake.setPower(0.7);
//                }
//            } else if(gamepad1.b){
//                intake.setPower(-0.7);
//            }   else {
//                intake.setPower(0);
//                }
//
//        }

    //            private void armControl () {
//                if (gamepad2.left_bumper) {
//                    runToPosition(ARM_SCORE_POSITION);
//                    manual = false;
//                } else if (gamepad2.right_bumper) {
//                    runToPosition(ARM_COLLECT_POSITION);
//                    manual = true;
//                }
//            }
    private void armPIDControl() {
        int armRPos = armRotatorRight.getCurrentPosition();
        int armLPos = armRotatorLeft.getCurrentPosition();

        double pid = controller.calculate(armRPos, target);

        double ff = Math.cos(Math.toRadians(target / tick_per_degree)) * f;

        double power = pid + ff;
        if(gamepad2.left_bumper) {
            target = -500;
            manual = false;
        } else if (gamepad2.right_bumper) {
            target = 1000;
            manual = true;
        }
        armRotatorLeft.setPower(power);
        armRotatorRight.setPower(power);
        telemetry.addData("posR", armRPos);
        telemetry.addData("posL", armLPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
    private void armManual () {
        double powerFor = gamepad1.left_trigger;
        double powerRev = gamepad1.right_trigger;
        if (Math.abs(powerFor) > 0.1 && armRotatorLeft.getCurrentPosition() < ARM_SCORE_POSITION && armRotatorRight.getCurrentPosition() < ARM_SCORE_POSITION) {
            armRotatorLeft.setPower((powerFor - powerRev) * 0.3);
            armRotatorRight.setPower((powerFor - powerRev) * 0.3);
        }
    }
//            private void wristControl () {
//                if (gamepad1.right_bumper) {
//                    wrist.setPosition(0.6);
//                } else if (gamepad1.left_bumper) {
//                    wrist.setPosition(0.7);
//                }
//            }

}



