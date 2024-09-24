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

    @Override
    public void init() {
       telemetry.addData("Status", "Initialized");
       x = hardwareMap.get(CRServoImpl.class, "x");
       y = hardwareMap.get(CRServoImpl.class, "y");
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        runtime.reset();
    }
    double z = 0;
    @Override
    public void loop() {
        if (gamepad1.a) {
            z = .2;
        }
        if (gamepad1.b) {
            z = .2;
        }
        x.setPower(z);
        y.setPower(z);
        z = 0;
//whynot
    }
    @Override
    public void stop() {
    }
}
