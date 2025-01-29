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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "rightSideAUTO", preselectTeleOp = "FIELDILT")
public class RightAutoILT extends LinearOpMode {

    public class armControl {
        private DcMotorEx armRotatorLeft;
        private DcMotorEx armRotatorRight;
//        private PIDController controller;
//
//        public  double p = 0.005, i = 0.0008, d = 0.0002;
//        public  double f = 0.0001;
//
//        public  int target = 0;
//        public final double TICKS_PER_DEGREE = 537.7 / 360;


        public  armControl(HardwareMap hardwareMap) {
            armRotatorLeft = hardwareMap.get(DcMotorEx.class, "armRL");
            armRotatorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armRotatorRight = hardwareMap.get(DcMotorEx.class, "armRR");
            armRotatorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armRotatorRight.setDirection(DcMotorEx.Direction.REVERSE);

            armRotatorLeft.setDirection(DcMotorEx.Direction.FORWARD);
            armRotatorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            armRotatorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            armRotatorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            armRotatorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        }

        public class armScoreBar implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on


                if (!initialized) {
                    armRotatorLeft.setPower(0.75);
                    armRotatorRight.setPower(0.75);

//                    armRotatorLeft.setTargetPosition(target);
//                    armRotatorRight.setTargetPosition(target);
//                    armRotatorRight.setMode((DcMotor.RunMode.RUN_TO_POSITION));
//                    armRotatorLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));

                    initialized = true;
                }
                // checks lift's current position


                // checks lift's current position


                int   target = 100;



                armRotatorLeft.setTargetPosition(target);
                armRotatorRight.setTargetPosition(target);
                armRotatorRight.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                armRotatorLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));

                double posR = armRotatorRight.getCurrentPosition();
                double posL = armRotatorLeft.getCurrentPosition();
                double error = (posR+posL)/2-target;

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
        public Action ArmScoreBar() {
            return new armScoreBar();
        }
        public class armTouch implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on


                if (!initialized) {
                    armRotatorLeft.setPower(0.75);
                    armRotatorRight.setPower(0.75);

//                    armRotatorLeft.setTargetPosition(target);
//                    armRotatorRight.setTargetPosition(target);
//                    armRotatorRight.setMode((DcMotor.RunMode.RUN_TO_POSITION));
//                    armRotatorLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));

                    initialized = true;
                }
                // checks lift's current position


                // checks lift's current position


                int   target = 100;



                armRotatorLeft.setTargetPosition(target);
                armRotatorRight.setTargetPosition(target);
                armRotatorRight.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                armRotatorLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));

                double posR = armRotatorRight.getCurrentPosition();
                double posL = armRotatorLeft.getCurrentPosition();
                double error = (posR+posL)/2-target;

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
        public Action ArmTouch() {
            return new armTouch();
        }
        public class armCollect implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on

                if (!initialized) {
                    armRotatorLeft.setPower(0.75);
                    armRotatorRight.setPower(0.75);
                    initialized = true;
                }


                int armRPos = armRotatorRight.getCurrentPosition();
                int armLPos = armRotatorLeft.getCurrentPosition();



                int target = 1000;





                armRotatorLeft.setTargetPosition(target);
                armRotatorRight.setTargetPosition(target);
                armRotatorRight.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                armRotatorLeft.setMode((DcMotor.RunMode.RUN_TO_POSITION));

                double posR = armRotatorRight.getCurrentPosition();
                double posL = armRotatorLeft.getCurrentPosition();
                double error = (posR+posL)/2-target;
                // checks lift's current position


                if (error > -15 && error < 15) {
                    // true causes the action to rerun
                    armRotatorLeft.setPower(0);
                    armRotatorRight.setPower(0);
                    telemetry.addData("errorA",error);
                    telemetry.update();
                    return false;

                } else {
                    // false stops action rerun
                    return true;
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




        public slideControl(HardwareMap hardwareMap) {
            slideLeft = hardwareMap.get(DcMotorEx.class, "slideL");
            slideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            slideRight = hardwareMap.get(DcMotorEx.class, "slideR");
            slideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            slideLeft.setDirection(DcMotorEx.Direction.FORWARD);
            slideRight.setDirection(DcMotorEx.Direction.REVERSE);
            slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        public class slideScoreTopBasket implements Action {
            // checks if the lift motor has been powered on

            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                    initialized = true;
                }

                slideRight.setTargetPosition(2450);
                slideLeft.setTargetPosition(2450);
                slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // checks lift's current position
                double posR = slideRight.getCurrentPosition();
                double posL = slideLeft.getCurrentPosition();
                double error = (posR+posL)/2-2000;
                if (error > -15 && error < 15 ){

                    // true causes the action to rerun
                    slideLeft.setPower(0);
                    slideRight.setPower(0);
                    telemetry.addData("errorS",error);
                    telemetry.update();
                    return false;
                } else {
                    // false stops action rerun

                    return true;
                }
            }
        }
        public Action SlideScoreTopBasket() {
            return new slideScoreTopBasket();
        }
        public class slideCollect implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                    initialized = true;
                }
                // checks lift's current position
                double posR = slideRight.getCurrentPosition();
                double posL = slideLeft.getCurrentPosition();

                slideRight.setTargetPosition(810);
                slideLeft.setTargetPosition(810);
                slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // checks lift's current position
                double error = (posR+posL)/2 -810;


                if (error> -15 && error <15 ){
                    // true causes the action to rerun
                    slideLeft.setPower(0);
                    slideRight.setPower(0);
                    telemetry.addData("errorS",error);
                    telemetry.update();
                    return false;
                } else {
                    // false stops action rerun

                    return true;
                }
            }
        }
        public Action SlideCollect() {
            return new slideCollect();
        }
        public class slideDown implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                    initialized = true;
                }
                // checks lift's current position
                double posR = slideRight.getCurrentPosition();
                double posL = slideLeft.getCurrentPosition();

                slideRight.setTargetPosition(10);
                slideLeft.setTargetPosition(10);
                slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // checks lift's current position
                double error = (posR+posL)/2 -10;


                if (error> -15 && error <15 ){
                    // true causes the action to rerun
                    slideLeft.setPower(0);
                    slideRight.setPower(0);
                    return false;
                } else {
                    // false stops action rerun

                    return true;
                }
            }
        }
        public Action SlideDown() {
            return new slideDown();
        }
        public class slideTouch implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                    initialized = true;
                }
                // checks lift's current position
                double posR = slideRight.getCurrentPosition();
                double posL = slideLeft.getCurrentPosition();

                slideRight.setTargetPosition(100);
                slideLeft.setTargetPosition(100);
                slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // checks lift's current position
                double error = (posR+posL)/2 -100;


                if (error> -15 && error <15 ){
                    // true causes the action to rerun
                    slideLeft.setPower(0);
                    slideRight.setPower(0);
                    return false;
                } else {
                    // false stops action rerun

                    return true;
                }
            }
        }
        public Action SlideTouch() {
            return new slideTouch();
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

        armControl arm = new armControl(hardwareMap);
        slideControl slide = new slideControl(hardwareMap);
        wristControl wrist= new wristControl(hardwareMap);
        intakeControl intake = new intakeControl(hardwareMap);

        int visionOutputPosition = 1;

        Action tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-63, -58, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(1)

                .build();

        Action tab2 = drive.actionBuilder(new Pose2d(-63, -58, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(-52.5, -50, Math.toRadians(90)), Math.toRadians(90))
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






//                .splineToLinearHeading(new Pose2d(-47.5, -50, Math.toRadians(90)), Math.toRadians(90))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)), Math.toRadians(45))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearHeading(new Pose2d(-58, -33, Math.toRadians(90)), Math.toRadians(90))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(45))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-57.5, -50, Math.toRadians(115)), Math.toRadians(115))//.splineToLinearHeading(new Pose2d(-58, -33, Math.toRadians(90)), Math.toRadians(90))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(45))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(-25, -12, Math.toRadians(0)), Math.toRadians(0))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
//                .waitSeconds(1)

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        tab1,
                        arm.ArmScoreBasket(),
                        slide.SlideScoreTopBasket(),
                        slide.SlideDown(),
//                        wrist.closewrist(),
//                        intake.openintake(),
                        tab2,
                        arm.ArmCollect(),
                        slide.SlideCollect(),
//                        wrist.openwrist(),
//                        intake.closeintake(),
                        arm.ArmScoreBasket(),
                        slide.SlideScoreTopBasket(),
                        tab3,
                        slide.SlideDown(),
//                        wrist.closewrist(),
//                        intake.openintake(),
                        tab4,
                        arm.ArmCollect(),
                        slide.SlideCollect(),
//                        wrist.openwrist(),
//                        intake.closeintake(),
                        arm.ArmScoreBasket(),
                        tab5,
                        slide.SlideScoreTopBasket(),
                        slide.SlideDown(),
//                        wrist.closewrist(),
//                        intake.openintake(),
                        tab6,
                        arm.ArmCollect(),
                        slide.SlideCollect(),
//                        wrist.openwrist(),
//                        intake.closeintake(),
                        arm.ArmScoreBasket(),
                        tab7,
                        slide.SlideScoreTopBasket(),
                        slide.SlideDown(),
//                        wrist.closewrist(),
//                        intake.openintake(),
                        tab8,
                        arm.ArmTouch(),



















                        new SleepAction(1)

//                        intake.openintake()
                )
        );

    }







}



