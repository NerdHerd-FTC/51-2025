package org.firstinspires.ftc.teamcode.AUTOS;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "leftSideAuto", group = "Autonomous")
public class LeftAUTOSPECIMEN extends LinearOpMode {

    public class armControl {
        private DcMotorEx armRotatorLeft;
        private DcMotorEx armRotatorRight;

        public  armControl(HardwareMap hardwareMap) {
            armRotatorLeft = hardwareMap.get(DcMotorEx.class, "armL");
            armRotatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armRotatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
            armRotatorRight = hardwareMap.get(DcMotorEx.class, "armR");
            armRotatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armRotatorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class armScoreBasket implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armRotatorLeft.setPower(0.8);
                    armRotatorRight.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double posR = armRotatorRight.getCurrentPosition();
                double posL = armRotatorLeft.getCurrentPosition();
                packet.put("armPosR", posR);
                packet.put("armPosL", posL);
                if (posR < 100.0 && posL <100) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    armRotatorRight.setPower(0);
                    armRotatorLeft.setPower(0);
                    return false;
                }
            }
        }
        public Action ArmScoreBasket() {
            return new armScoreBasket();
        }
        public class armCollect implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armRotatorLeft.setPower(-0.8);
                    armRotatorRight.setPower(-0.8);
                    initialized = true;
                }

                // checks lift's current position
                double posR = armRotatorRight.getCurrentPosition();
                double posL = armRotatorLeft.getCurrentPosition();
                packet.put("armPosR", posR);
                packet.put("armPosL", posL);
                if (posR > 5 && posL > 5) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    armRotatorRight.setPower(0);
                    armRotatorLeft.setPower(0);
                    return false;
                }
            }
        }
        public Action ArmCollect() {
            return new armCollect();
        }

    }
    public class slideControl {
        private DcMotorEx slideLeft;
        private DcMotorEx slideRight;

        public  slideControl(HardwareMap hardwareMap) {
            slideLeft = hardwareMap.get(DcMotorEx.class, "armL");
            slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
            slideRight = hardwareMap.get(DcMotorEx.class, "armR");
            slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        public class slideScoreTopBasket implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    slideLeft.setPower(0.8);
                    slideRight.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double posR = slideRight.getCurrentPosition();
                double posL = slideLeft.getCurrentPosition();
                packet.put("slidePosR", posR);
                packet.put("slidePosL", posL);
                if (posR < 100.0 && posL <100) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    slideRight.setPower(0);
                    slideLeft.setPower(0);
                    return false;
                }
            }
        }
        public class slideLower implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    slideLeft.setPower(-0.8);
                    slideRight.setPower(-0.8);
                    initialized = true;
                }

                // checks lift's current position
                double posR = slideRight.getCurrentPosition();
                double posL = slideLeft.getCurrentPosition();
                packet.put("slidePosR", posR);
                packet.put("slidePosL", posL);
                if (posR > 100.0 && posL >100) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    slideRight.setPower(0);
                    slideLeft.setPower(0);
                    return false;
                }
            }
        }
        public Action SlideLower() {
            return new slideControl.slideLower();
        }

    }
    public class wristControl {
        private Servo wrist;


        public  wristControl(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "wrist");
        }
        public class Closewrist implements Action {

            @Override

            public boolean run(@NonNull TelemetryPacket packet) {

                wrist.setPosition(0.55);

                return false;

            }

        }

        public Action closewrist() {

            return new Closewrist();

        }



        public class Openwrist implements Action {

            @Override

            public boolean run(@NonNull TelemetryPacket packet) {

                wrist.setPosition(1.0);

                return false;

            }

        }

        public Action openwrist() {

            return new Openwrist();

        }



    }
    public class intakeControl {
        private Servo intake;


        public intakeControl(HardwareMap hardwareMap) {
            intake = hardwareMap.get(Servo.class, "intake");

        }
        public class Closeintake implements Action {

            @Override

            public boolean run(@NonNull TelemetryPacket packet) {

                intake.setPosition(0.55);

                return false;

            }

        }

        public Action closeintake() {

            return new Closeintake();

        }



        public class Openintake implements Action {

            @Override

            public boolean run(@NonNull TelemetryPacket packet) {

                intake.setPosition(1.0);

                return false;

            }

        }

        public Action openintake() {

            return new Openintake();
        }
    }
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-47, -63.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        armControl armRotatorRight = new armControl(hardwareMap);
        armControl armRotatorLeft = new armControl(hardwareMap);
        slideControl slideRight = new slideControl(hardwareMap);
        slideControl slideLeft = new slideControl(hardwareMap);
        wristControl wrist= new wristControl(hardwareMap);
        intakeControl intake = new intakeControl(hardwareMap);

        int visionOutputPosition = 1;

        Action tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-52, -63.5, Math.toRadians(90)), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(-58, -33, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(-66, -33, Math.toRadians(120)), Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
                .build();

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        tab1,
                        new SleepAction(1),
                        intake.openintake()
                )
        );

    }







}


