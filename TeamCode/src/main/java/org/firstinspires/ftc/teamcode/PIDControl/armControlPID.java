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

@Config
@TeleOp
public class armControlPID extends OpMode {

    private PIDController controller;

    public static double p = 0.005, i = 0.0008, d = 0.0002;
    public static double f = 0.0001;

    public static int target = 0;

    private final double tick_per_degree = 537.7/360;

    private DcMotorEx armR;
    private DcMotorEx armL;



    @Override
    public void init() {
        controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armR = hardwareMap.get(DcMotorEx.class, "armRR");
        armL = hardwareMap.get(DcMotorEx.class, "armRL");

        armR.setDirection(DcMotorSimple.Direction.REVERSE);
        armL.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armRPos = armR.getCurrentPosition();
        int armLPos = armL.getCurrentPosition();

        double pid = controller.calculate(armRPos, target);

        double ff = Math.cos(Math.toRadians(target / tick_per_degree)) * f;

        double power = pid + ff;
        if(gamepad1.a) {
            target = 580;
        } else if (gamepad1.b) {
            target = 1000;
        }
        armR.setPower(power);
        armL.setPower(power);
        telemetry.addData("posR", armRPos);
        telemetry.addData("posL", armLPos);
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
