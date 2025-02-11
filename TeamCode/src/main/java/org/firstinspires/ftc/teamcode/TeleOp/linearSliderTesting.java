package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "armTesting")
public class linearSliderTesting extends LinearOpMode {
    private DcMotor armRotatorRight;

    private DcMotor armRotatorLeft;

    private DcMotorEx slideRight;

    private DcMotorEx slideLeft;
    public int targetPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        armRotatorRight = hardwareMap.dcMotor.get("armRR");
        armRotatorLeft= hardwareMap.dcMotor.get("armRL");
        slideRight = (DcMotorEx) hardwareMap.dcMotor.get("slideR");
        slideLeft = (DcMotorEx) hardwareMap.dcMotor.get("slideL");

        armRotatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armRotatorLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        armRotatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Main control loop
            int armRPos = armRotatorRight.getCurrentPosition();
            int armLPos = armRotatorLeft.getCurrentPosition();
            int slideRPos = slideRight.getCurrentPosition();
            int slideLPos = slideLeft.getCurrentPosition();
            telemetry.addData("posR", slideRPos);
            telemetry.addData("posL", slideLPos);
            telemetry.addData("posR", armRPos);
            telemetry.addData("posL", armLPos);


            telemetry.update();

            // telemetry.addData("wrist", wrist.getPosition());


        }

    }







}
