package org.firstinspires.ftc.teamcode.PIDControl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@Config
@TeleOp
public class armControlPID extends OpMode {

    private PIDController controller;

    public static double p = 0.00, i = 0.000, d = 0.000;
    public static double f = 0.000;

    public static int target ;

    private final double tick_per_degree = 537.7/360;

    private DcMotorEx armR;
    private DcMotorEx armL;

    private DcMotorEx slideRight;

    private DcMotorEx slideLeft;



    @Override
    public void init() {
        controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armR = hardwareMap.get(DcMotorEx.class, "armRR");
        armL = hardwareMap.get(DcMotorEx.class, "armRL");
        slideRight = (DcMotorEx) hardwareMap.dcMotor.get("slideR");
        slideLeft = (DcMotorEx) hardwareMap.dcMotor.get("slideL");

        armR.setDirection(DcMotorSimple.Direction.REVERSE);
        armL.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armRPos = armR.getCurrentPosition();
        int armLPos = armL.getCurrentPosition();
        int slideRPos = slideRight.getCurrentPosition();
        int slideLPos = slideLeft.getCurrentPosition();

        int pos = (int)(armRPos+armLPos+slideRPos+slideLPos)/4;

        double pid = controller.calculate(pos, target);

        double ff = Math.cos(Math.toRadians(target / tick_per_degree)) * f;

        double power = pid + ff;

        armR.setPower(power);
        armL.setPower(power);
        slideRight.setPower(power);
        slideLeft.setPower(power);
        telemetry.addData("posR", armRPos);
        telemetry.addData("posL", armLPos);
        telemetry.addData("posR", slideRPos);
        telemetry.addData("posL", slideLPos);


        telemetry.addData("target", target);
        telemetry.update();
    }
}
//package org.firstinspires.ftc.teamcode.PIDControl;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//@Config
//@TeleOp
//public class armControlPID extends LinearOpMode {
//        DcMotorEx armR;
//    DcMotorEx armL;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//    armR = hardwareMap.get(DcMotorEx.class, "armR");
//        armL = hardwareMap.get(DcMotorEx.class, "armR");
//        waitForStart();
//        while(opModeIsActive()) {
//
//        }
//    }
//}
//
