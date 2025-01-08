package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "colorSensing")
public class ColorSensingTest extends LinearOpMode {
    private ColorSensor colorSensor;

    private CRServo intake;

    private double blueValue;
    private double greenValue;
    private double redValue;

    private double alphaValue; // light intensity

    private double target = 1000;


    @Override
    public void runOpMode() throws InterruptedException {
    colorSensor = hardwareMap.get(ColorSensor.class, "colorV3");

      while(!isStarted()) {
          blueValue = colorSensor.blue();
          greenValue = colorSensor.green();
          redValue = colorSensor.red();
          alphaValue = colorSensor.alpha();
          telemetry.addData("redvalue","%.2f", redValue );
          telemetry.addData("bluevalue","%.2f", blueValue );
          telemetry.addData("greenvalue","%.2f", greenValue );
          telemetry.addData("alphavalue","%.2f", alphaValue );
          telemetry.update();
          intake = hardwareMap.crservo.get("intake");
      }
      waitForStart();
      while(opModeIsActive()) {
          blueValue = colorSensor.blue();
          greenValue = colorSensor.green();
          redValue = colorSensor.red();
          alphaValue = colorSensor.alpha();
          telemetry.addData("redvalue","%.2f", redValue );
          telemetry.addData("bluevalue","%.2f", blueValue );
          telemetry.addData("greenvalue","%.2f", greenValue );
          telemetry.addData("alphavalue","%.2f", alphaValue );
          telemetry.update();
          if (gamepad1.a) {
              if (blueValue > 2500 && greenValue < 1500 && redValue < 1500) {
                  intake.setPower(0);
              } else if (blueValue < 300 && greenValue > 700 && redValue > 700) {
                  intake.setPower(0);
              } else if (blueValue < 2500 && greenValue < 1500 && redValue > 2500) {
                  intake.setPower(0.3);
              } else {
                  intake.setPower(1);
              }
          } else if(gamepad1.b){
              intake.setPower(-1);
          }   else {
              intake.setPower(0);
          }

      }
    }

}

