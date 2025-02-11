package org.firstinspires.ftc.teamcode.AUTOS;




import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.acmerobotics.roadrunner.Action;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "leftSideAuto", preselectTeleOp = "FIELDILT")
public class LeftSampleAutoBUP extends LinearOpMode {

    public class slideControl {
        private DcMotorEx armRotatorLeft;
        private DcMotorEx armRotatorRight;

        private DcMotorEx slideRight;

        private DcMotorEx slideLeft;



        public  slideControl(HardwareMap hardwareMap) {
            armRotatorLeft = hardwareMap.get(DcMotorEx.class, "armRL");
            armRotatorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armRotatorRight = hardwareMap.get(DcMotorEx.class, "armRR");
            armRotatorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armRotatorRight.setDirection(DcMotorEx.Direction.REVERSE);

            slideRight = (DcMotorEx) hardwareMap.dcMotor.get("slideR");
            slideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            slideRight.setDirection(DcMotorEx.Direction.REVERSE);
            slideLeft = (DcMotorEx) hardwareMap.dcMotor.get("slideL");
            slideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            slideLeft.setDirection(DcMotorEx.Direction.REVERSE);

            armRotatorLeft.setDirection(DcMotorEx.Direction.FORWARD);


            armRotatorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            armRotatorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            armRotatorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            armRotatorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        }

        public class slideScoreBasket implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on


                if (!initialized) {
                    armRotatorLeft.setPower(0.75);
                    armRotatorRight.setPower(0.75);
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);


//                    armRotatorLeft.setTargetPosition(target);
//                    armRotatorRight.setTargetPosition(target);
//                    armRotatorRight.setMode((DcMotor.RunMode.RUN_TO_POSITION));
//                    armRotatorLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));

                    initialized = true;
                }
                // checks lift's current position


                // checks lift's current position


                int   target = 500;



                armRotatorLeft.setTargetPosition(target);
                armRotatorRight.setTargetPosition(target);
                slideLeft.setTargetPosition(target);
                slideRight.setTargetPosition(target);

                armRotatorRight.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                armRotatorLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                slideRight.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                slideLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));

                double posR = armRotatorRight.getCurrentPosition();
                double posL = armRotatorLeft.getCurrentPosition();
                double posSR = slideRight.getCurrentPosition();
                double posSL = slideLeft.getCurrentPosition();

                double error = (posR+posL+posSR+posSL)/4-target;

                if (error > -15 && error < 15 ) {

                    // true causes the action to rerun
                    armRotatorLeft.setPower(0);
                    armRotatorRight.setPower(0);
                    telemetry.addData("errorA",error);
                    telemetry.update();
                    return false;
                } else {

                    return true;
                }
            }
        }
        public Action SlideScoreBasket() {
            return new slideScoreBasket();
        }
        public class slideDown implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on


                if (!initialized) {
                    armRotatorLeft.setPower(0.75);
                    armRotatorRight.setPower(0.75);
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);


//                    armRotatorLeft.setTargetPosition(target);
//                    armRotatorRight.setTargetPosition(target);
//                    armRotatorRight.setMode((DcMotor.RunMode.RUN_TO_POSITION));
//                    armRotatorLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));

                    initialized = true;
                }
                // checks lift's current position


                // checks lift's current position


                int   target = 0;



                armRotatorLeft.setTargetPosition(target);
                armRotatorRight.setTargetPosition(target);
                slideLeft.setTargetPosition(target);
                slideRight.setTargetPosition(target);

                armRotatorRight.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                armRotatorLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                slideRight.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                slideLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));

                double posR = armRotatorRight.getCurrentPosition();
                double posL = armRotatorLeft.getCurrentPosition();
                double posSR = slideRight.getCurrentPosition();
                double posSL = slideLeft.getCurrentPosition();

                double error = (posR+posL+posSR+posSL)/4-target;

                if (error > -15 && error < 15 ) {

                    // true causes the action to rerun
                    armRotatorLeft.setPower(0);
                    armRotatorRight.setPower(0);
                    telemetry.addData("errorA",error);
                    telemetry.update();
                    return false;
                } else {

                    return true;
                }
            }
        }
        public Action Slide() {
            return new slideDown();
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
    public class grabControl {
        private Servo grabR;
        private Servo grabL;

        public  grabControl(HardwareMap hardwareMap) {
            grabR = hardwareMap.servo.get("grabR");
            grabL = hardwareMap.servo.get("grabL");
        }
        public class extendGrab implements Action {

            @Override

            public boolean run(@NonNull TelemetryPacket packet) {

                grabR.setPosition(.7);
                grabL.setPosition(.7);

                return false;

            }

        }

        public Action ExtendGrab() {

            return new extendGrab();

        }



        public class retractGrab implements Action {

            @Override

            public boolean run(@NonNull TelemetryPacket packet) {

                grabR.setPosition(.15);
                grabL.setPosition(.15);

                return false;

            }

        }

        public Action RetractGrab() {

            return new retractGrab();

        }



    }
    public class armControl {
        private Servo armR;
        private Servo armL;
        public  armControl(HardwareMap hardwareMap) {
            armR = hardwareMap.servo.get("armR");
            armL = hardwareMap.servo.get("armL");
        }
        public class armCollect implements Action {

            @Override

            public boolean run(@NonNull TelemetryPacket packet) {

                armR.setPosition(.7);
                armL.setPosition(.7);

                return false;

            }

        }

        public Action ArmCollect() {

            return new armCollect();

        }



        public class armScoreBasket implements Action {

            @Override

            public boolean run(@NonNull TelemetryPacket packet) {

                armR.setPosition(.15);
                armL.setPosition(.15);

                return false;

            }

        }

        public Action ArmScoreBasket() {

            return new armScoreBasket();

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
        Pose2d initialPose = new Pose2d(-35.20 , -64.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        slideControl slide = new slideControl(hardwareMap);

        wristControl wrist= new wristControl(hardwareMap);
        intakeControl intake = new intakeControl(hardwareMap);
        armControl arm = new armControl(hardwareMap);
        grabControl grab = new grabControl(hardwareMap);


        int visionOutputPosition = 1;

        Action tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-55.54, -56.00, Math.toRadians(45.57)), Math.toRadians(152.57))

                .waitSeconds(1)

                .build();

        Action tab2 = drive.actionBuilder(new Pose2d(-55.54, -56, Math.toRadians(45.57)))
                .splineToLinearHeading(new Pose2d(-47.69, -45.71, Math.toRadians(90.00)), Math.toRadians(61.16))

                .waitSeconds(1)
                .build();
        Action tab3 = drive.actionBuilder(new Pose2d(-52.5, -50, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-63, -58, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(1)
                .build();
        Action tab4 = drive.actionBuilder(new Pose2d(-63, -58, Math.toRadians(45)))
//                .splineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(0)), Math.toRadians(0))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
//                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-56.5, -50, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)
                .build();
        Action tab5 = drive.actionBuilder(new Pose2d(-56.5, -50, Math.toRadians(90)))
//                .splineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(0)), Math.toRadians(0))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
//                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-63, -58, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(1)
                .build();
        Action tab6 = drive.actionBuilder(new Pose2d(-63, -58, Math.toRadians(45)))
//
                .splineToLinearHeading(new Pose2d(-60.5, -50, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)
                .build();
        Action tab7 = drive.actionBuilder(new Pose2d(-60.5, -50, Math.toRadians(90)))
//                .splineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(0)), Math.toRadians(0))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
//                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-63, -58, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(1)
                .build();
        Action tab8 = drive.actionBuilder(new Pose2d(-63, -58, Math.toRadians(90)))
//                .splineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(0)), Math.toRadians(0))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
//                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(0)), Math.toRadians(0))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
                .waitSeconds(1)
                .build();






//.splineToLinearHeading(new Pose2d(-55.54, -56.00, Math.toRadians(45.57)), Math.toRadians(152.57))
//
//                .waitSeconds(1)
//
//
//
//                .splineToLinearHeading(new Pose2d(-47.69, -45.71, Math.toRadians(90.00)), Math.toRadians(61.16))
//
//                .waitSeconds(1)
//
//                .splineToLinearHeading(new Pose2d(-55.54, -56.00, Math.toRadians(45.57)), Math.toRadians(152.57))
//
//
//                .waitSeconds(1)
//
//
//                .splineToLinearHeading(new Pose2d(-58.20, -45.94, Math.toRadians(90.00)), Math.toRadians(188.32))
//
//                .waitSeconds(1)
//
//                .splineToLinearHeading(new Pose2d(-55.54, -56.00, Math.toRadians(45.57)), Math.toRadians(152.57))
//
//
//                .waitSeconds(1)
//
//                .splineToLinearHeading(new Pose2d(-57.66, -42.74, Math.toRadians(127.00)), Math.toRadians(98.13))
//
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-55.54, -56.00, Math.toRadians(45.57)), Math.toRadians(152.57))
//
//
//                .waitSeconds(1)
//
//
//                .splineTo(new Vector2d(-52.80, -16.91), Math.toRadians(42.95))
//
//                .splineToLinearHeading(new Pose2d(-24.69, -8.69, Math.toRadians(0.00)), Math.toRadians(53.97))
//                .build();

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        tab1,


//                        wrist.closewrist(),
//                        intake.openintake(),
                        tab2,


//                        wrist.openwrist(),
//                        intake.closeintake(),


                        tab3,

//                        wrist.closewrist(),
//                        intake.openintake(),
                        tab4,


//                        wrist.openwrist(),
//                        intake.closeintake(),

                        tab5,

//                        wrist.closewrist(),
//                        intake.openintake(),
                        tab6,


//                        wrist.openwrist(),
//                        intake.closeintake(),

                        tab7,

//                        wrist.closewrist(),
//                        intake.openintake(),
                        tab8,




















                        new SleepAction(1)

//                        intake.openintake()
                )
        );

    }







}



