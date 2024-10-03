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
    private ServoImpl x = null;
    private ServoImpl y  = null;
    private DcMotor z = null;
    private DcMotor rf = null;
    private DcMotorSimple lf = null;
    private DcMotor rb = null;
    private DcMotor lb = null;

    @Override
    public void init() {
       telemetry.addData("Status", "Initialized");
       x = hardwareMap.get(ServoImpl.class, "x");
       y = hardwareMap.get(ServoImpl.class, "y");
       z = hardwareMap.get(DcMotor.class, "z");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lf = hardwareMap.get(DcMotorSimple.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

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
    double v = 0;
    boolean t = false;
    boolean o = false;
    boolean u = false;
    boolean i = false;
    double rp;
    double lp;
    @Override
    public void loop() {

        double sm = 2.30 - (gamepad1.right_bumper ? 1 : 0) + (gamepad1.left_bumper ? 1 : 0);
        if (gamepad1.right_stick_x > 0) {
            sm -= 0.75;

        }
        sm = 1.9;
        lp = (-gamepad1.left_stick_y / sm) + (-gamepad1.right_stick_x / sm);
        rp = (-gamepad1.left_stick_y / sm) + (gamepad1.right_stick_x / sm);

        if (gamepad2.left_bumper) {
            lb.setPower(0);
            lf.setPower(0);
            rb.setPower(0);
            rf.setPower(0);
        }else if (gamepad1.left_stick_x >= 1) {
            lb.setPower(-0.6);
            lf.setPower(0.6);
            rb.setPower(0.6);
            rf.setPower(-0.6);
        }else if (gamepad1.left_stick_x <= -1) {
            lb.setPower(0.6);
            lf.setPower(-0.6);
            rb.setPower(-0.6);
            rf.setPower(0.6);
        }else {
            lb.setPower(rp);
            lf.setPower(rp);
            rb.setPower(lp);
            rf.setPower(lp);
        }


        if (gamepad1.left_bumper) {
            g = .3;
        }
        if (gamepad1.right_bumper) {
            g = -.3;
        }
        z.setPower(g);
        g = 0;


        if (gamepad1.dpad_up && t == false){
            v += .1;
            t = true;
        }
        if (gamepad1.dpad_up != true){
            t = false;
        }


        if (gamepad1.dpad_down && o == false){
            v -= .1;
            o = true;
        }
        if (gamepad1.dpad_down != true){
            o = false;
        }


        if (gamepad1.dpad_left && u == false){
            k += .1;
            u = true;

        }
        if (gamepad1.dpad_left != true){
            u = false;
        }

        
        if (gamepad1.dpad_right && i == false){
            k -= .1;
            i = true;
        }
        if (gamepad1.dpad_right != true){
            i = false;
        }

        if (gamepad1.x){
            x.setPosition(v);

        }
        if (gamepad1.y){
            y.setPosition(k);
        }
        telemetry.addData("Status", "x position: " + v);
        telemetry.addData("Status2", "y position: " + k);
        

//whynot
    }
    @Override
    public void stop() {
    }
}
