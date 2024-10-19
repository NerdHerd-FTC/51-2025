import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "armTesting")
public class linearSliderTesting extends OpMode {

    @Override
    public void init() {
        DcMotor leftSlide = hardwareMap.dcMotor.get("leftSlide");
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.getCurrentPosition();

        leftSlide.isBusy();

    }

    @Override
    public void loop() {

    }
}
