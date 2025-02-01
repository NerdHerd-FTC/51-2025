package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

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
@TeleOp(name = "intakeTest")
@Config
public class RollerIntakeTesting extends LinearOpMode {
    private CRServo rollerR;
    private CRServo rollerL;

    @Override
    public void runOpMode() throws InterruptedException {
        rollerR = hardwareMap.crservo.get("rollerR");
        rollerL = hardwareMap.crservo.get("rollerL");
        rollerR.setDirection(CRServo.Direction.REVERSE);
        rollerL.setDirection(CRServo.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {

            intakeControl();


        }

    }
    private void intakeControl() {


        if (gamepad1.a) {
            rollerR.setPower(0.7);
            rollerL.setPower(-0.7);

        } else if(gamepad1.b){
            rollerR.setPower(-0.4);
            rollerL.setPower(0.4);
        }   else {
            rollerR.setPower(0);
            rollerL.setPower(0);
        }


    }

}
