package org.firstinspires.ftc.teamcode.ROBOTCONTROLS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "CR_Servo")
@Config
public class CRServoTesting extends LinearOpMode {

     private CRServo rollerR;

    @Override
    public void runOpMode() throws InterruptedException {
        rollerR = hardwareMap.crservo.get("rollerR");
        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            if(gamepad1.a) {
            rollerR.setPower(-0.7);
            } else if (gamepad1.b) {
                rollerR.setPower(0.7);
            } else {
                rollerR.setPower(0);
            }



        }
    }
}

