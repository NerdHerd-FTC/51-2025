package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "armTesting")
public class linearSliderTesting extends LinearOpMode {
    private DcMotor slideRight;

    private DcMotor slideLeft;
    public int targetPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        slideLeft = hardwareMap.dcMotor.get("slideL");
        slideRight = hardwareMap.dcMotor.get("slideR");

        slideLeft.setDirection(DcMotorEx.Direction.FORWARD);
        slideRight.setDirection((DcMotorEx.Direction.REVERSE));

        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Main control loop
            telemetry.addData("SlideL", slideLeft.getCurrentPosition());
            telemetry.addData("SlideR", slideRight.getCurrentPosition());

            // telemetry.addData("wrist", wrist.getPosition());
            telemetry.update();
            slideManual();
        }

    }
    private void slideManual() {
        if(slideLeft.getCurrentPosition() < 810 && slideRight.getCurrentPosition() <810 && slideLeft.getCurrentPosition() > -5 && slideRight.getCurrentPosition() > -5 ){
            if(gamepad2.a) {
               targetPosition = targetPosition + 30;

           } else if (gamepad2.b) {
                targetPosition = targetPosition - 30;
            } else {
                targetPosition = (slideRight.getCurrentPosition()+slideLeft.getCurrentPosition())/2 ;
           }
        } else if(slideLeft.getCurrentPosition() < 810 && slideRight.getCurrentPosition() <810 && !(slideLeft.getCurrentPosition() > -5 && slideRight.getCurrentPosition() > -5)) {
            if (gamepad2.b) {
                targetPosition = targetPosition - 30;
            } else {
                targetPosition = (slideRight.getCurrentPosition()+slideLeft.getCurrentPosition())/2 ;
            }
            if (gamepad2.a) {
                targetPosition = targetPosition + 30;
            } else {
                targetPosition = (slideRight.getCurrentPosition() + slideLeft.getCurrentPosition()) / 2;
            }
        } else if(!(slideLeft.getCurrentPosition() < 810 && slideRight.getCurrentPosition() <810) && slideLeft.getCurrentPosition() > -5 && slideRight.getCurrentPosition() > -5) {
            if (gamepad2.b) {
                targetPosition = targetPosition - 30;
            } else {
                targetPosition = (slideRight.getCurrentPosition()+slideLeft.getCurrentPosition())/2 ;
            }
        }   else {
            targetPosition = (slideRight.getCurrentPosition() + slideLeft.getCurrentPosition()) / 2;
        }
//
        slideRight.setPower(0.5);
        slideRight.setTargetPosition(targetPosition);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideLeft.setPower(0.5);
        slideLeft.setTargetPosition(targetPosition);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }






}
