package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "aaditesting")
public class Meet0Code extends OpMode {
    DcMotor motor;
    @Override
    public void init() {
        telemetry.addData("Initialization: ", "Is complete for Aadi");
        telemetry.update();
        motor = hardwareMap.get(DcMotor.class, "motor");
    }

    @Override
    public void loop() {
    if(gamepad1.a) {
        motor.setPower(0.5);
        telemetry.addData("Button A:", "Works");

    } else {
        motor.setPower(0.0);
        }
    telemetry.update();
    }






}
