package org.firstinspires.ftc.teamcode.ROBOTCONTROLS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servoTesting")
@Config
public class ServoTesting extends LinearOpMode {

    private Servo armR;

    private Servo armL;
    @Override
    public void runOpMode() throws InterruptedException {
        armR = hardwareMap.servo.get("claw");
       // armL = hardwareMap.servo.get("claaw");
        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            if(gamepad1.a) {
                armR.setPosition(0);
          //      armL.setPosition(1);
            } else if (gamepad1.b) {
                armR.setPosition(0.9);
            //    armR.setPosition(0.1);
            } else if(gamepad1.x) {
                armR.setPosition(0.8);
              //  armL.setPosition(0.8);
            } else if(gamepad1.y) {
                armR.setPosition(0.7);
                //armR.setPosition(0.7);
            } else if(gamepad2.a) {
                armR.setPosition(0.6);
               // armL.setPosition(0.6);
            } else if(gamepad2.b) {
                armR.setPosition(0.5);
               // armL.setPosition(0.5);
            } else if(gamepad2.x) {
                armR.setPosition(0.4);
                //armR.setPosition(0.6);
            } else if(gamepad2.y) {
                armR.setPosition(0.3);
                //armR.setPosition(0.7);

            } else if(gamepad1.right_bumper) {
               // armL.setPosition(0.135);
                armR.setPosition(.2);
            } else if(gamepad1.left_bumper) {
              //  armL.setPosition(0.1);
                armR.setPosition(0.1);
            } else if (gamepad2.right_bumper) {
                armR.setPosition(1);
              //  armL.setPosition(0);
            }
            double pos =   armR.getPosition();
            telemetry.addData("pos",pos);
            telemetry.update();
        }
    }
}
