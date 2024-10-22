package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Grace Test", group="Titans TeleOps")
public class Grace_Test extends OpMode {
    public static final double LINEAR_SLIDER_POSITION_MAX = 3999;
    public static final double LINEAR_SLIDER_POSITION_MIN = 11;
    private DcMotor linearSlide1 = null;
    private DcMotor linearSlide2 = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        linearSlide1 = hardwareMap.get(DcMotor.class, "linearSlide1");
        linearSlide1.setDirection(DcMotor.Direction.FORWARD);
        linearSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide2 = hardwareMap.get(DcMotor.class, "linearSlide2");
        linearSlide2.setDirection(DcMotor.Direction.FORWARD);
        linearSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        runtime.reset();
        telemetry.addLine("Boo!");
        telemetry.addLine("Heehee");
    }

    @Override
    public void loop() {
        double position = linearSlide1.getCurrentPosition();
        double linearSlidePower = 0;
        if (gamepad2.dpad_up && position < LINEAR_SLIDER_POSITION_MAX) {
            linearSlidePower = 1;
        }
        else if (gamepad2.dpad_down && position > LINEAR_SLIDER_POSITION_MIN) {
            linearSlidePower = -1;
        }
        else {
            linearSlidePower = 0;
        }
        if (position < LINEAR_SLIDER_POSITION_MIN) {
            linearSlidePower = 0;
        }
        if (position > LINEAR_SLIDER_POSITION_MAX) {
            linearSlidePower = 0;
        }
        linearSlide1.setPower(linearSlidePower);
        linearSlide2.setPower(linearSlidePower);
        telemetry.addData("Linear Slide", position);
    }
}