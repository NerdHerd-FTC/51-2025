import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.robot.Robot;
import com.sun.tools.javac.comp.Check;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "MeetOneTestingCode")
public class MeetOneTestingCode extends LinearOpMode {
    public CRServo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        DcMotor slideRight = hardwareMap.dcMotor.get("slideRight");
        DcMotor slideLeft = hardwareMap.dcMotor.get("slideLeft");

        DcMotor armRotatorRight = hardwareMap.dcMotor.get("rightRotate");
        DcMotor armRotatorLeft = hardwareMap.dcMotor.get("leftRotator");

       servo = hardwareMap.get(CRServo.class, "intake");
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
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

      slideRight.setTargetPosition(0);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setTargetPosition(0);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armRotatorRight.setTargetPosition(0);
       armRotatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       armRotatorLeft.setTargetPosition(0);
        armRotatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        double ticks_per_degree = 537.7 / 360;
        double ARM_SCORE = ticks_per_degree * 110;
        double ARM_COLLECT = armRotatorLeft.getCurrentPosition();
        double armPosition;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Retrieve the IMU from the hardware map
            IMU imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
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
                double backLeftPower = -(rotY - rotX + rx) / denominator;
                double frontRightPower = -(rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

                if(gamepad2.left_bumper) {
                    armPosition = ARM_SCORE;
                    armRotatorLeft.setTargetPosition((int) armPosition);
                    ((DcMotorEx) armRotatorLeft).setVelocity(1000);
                    armRotatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (armRotatorLeft.getCurrentPosition() == ((int) armPosition)) {
                        ((DcMotorEx) armRotatorLeft).setVelocity(0);
                    }
                } else if (gamepad2.right_bumper) {
                    armPosition = ARM_COLLECT;
                    armRotatorLeft.setTargetPosition((int) armPosition);
                    ((DcMotorEx) armRotatorLeft).setVelocity(1000);
                    armRotatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (armRotatorLeft.getCurrentPosition() == ((int) armPosition)) {
                        ((DcMotorEx) armRotatorLeft).setVelocity(0);
                    }
                }

                ((DcMotorEx) armRotatorRight).setVelocity(1750);

                armRotatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                if (gamepad1.a) {
                    servo.setPower(1);
                } else if (gamepad1.b) {
                    servo.setPower(-0.5);
                } else {
                    servo.setPower(0);
                }


            }
        }
    }
}