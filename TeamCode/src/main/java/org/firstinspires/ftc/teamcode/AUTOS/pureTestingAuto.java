package org.firstinspires.ftc.teamcode.AUTOS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Test Meet0 Auto0", group="Robot")

public class pureTestingAuto extends LinearOpMode {

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        int time = 0;

        waitForStart();

        while (time <= 15) {
            time++;
            Thread.sleep(1000);
            driftRight(0.6);
            //Thread.sleep(1000);
            //wait(1000);
        }
        time = 0;
       /* while (time ) {
            time++;
            Thread.sleep(1000);
            driftRight(0.6);
            //Thread.sleep(1000);
            wait(1000);
        } */


    }

    private void driftLeft(double speed) {
        frontLeftMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(-speed);
    }
    private void driftRight(double speed) {
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(speed);
    }

}

