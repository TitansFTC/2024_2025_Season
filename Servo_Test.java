package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Servo Test", group="Titans TeleOps")
public class Servo_Test extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo x = null;
    private CRServo y = null;
    private DcMotor z = null;

    @Override
    public void init() {
       telemetry.addData("Status", "Initialized");
       x = hardwareMap.get(CRServoImpl.class, "x");
       y = hardwareMap.get(CRServoImpl.class, "y");
       z = hardwareMap.get(DcMotor.class, "z");
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        runtime.reset();
    }
    double k = 0;
    double s = .2;
    double g = 0;
    @Override
    public void loop() {
        if (gamepad1.x) {
            s = .5;
        }
        if (gamepad1.y) {
            s = .2;
        }
        if (gamepad1.a) {
            k = -s;
        }
        if (gamepad1.b) {
            k = s;
        }
        x.setPower(-k);
        y.setPower(k);
        k = 0;
        if (gamepad1.left_bumper) {
            g = .3;
        }
        if (gamepad1.right_bumper) {
            g = -.3;
        }
        z.setPower(g);
        g = 0;
//whynot
    }
    @Override
    public void stop() {
    }
}
