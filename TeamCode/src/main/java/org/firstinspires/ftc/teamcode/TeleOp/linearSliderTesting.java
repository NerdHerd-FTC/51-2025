package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "armTesting")
public class linearSliderTesting extends LinearOpMode {
    DcMotor leftSlide;

    @Override
    public void runOpMode() throws InterruptedException {
        leftSlide = hardwareMap.dcMotor.get("slideLeft");
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Main control loop
            double power = gamepad1.left_stick_y;
            leftSlide.setPower(power);
            telemetry.addData("Left Slide:", leftSlide.getCurrentPosition());
        }
    }






}
