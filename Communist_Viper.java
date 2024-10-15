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

@TeleOp(name="Communist_Viper", group="Titans TeleOps")
public class Communist_Viper extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ServoImpl x = null;
    private ServoImpl y  = null;
    //private DcMotor z = null;
    private DcMotor rf = null;
    private DcMotorSimple rb = null;
    private DcMotor lf = null;
    private DcMotor lb = null;
    private DcMotor ar = null;
    private DcMotor le = null;
    private DcMotor le2 = null;

    @Override
    public void init() {
       telemetry.addData("Status", "Initialized");
       x = hardwareMap.get(ServoImpl.class, "x");
       y = hardwareMap.get(ServoImpl.class, "y");
       //z = hardwareMap.get(DcMotor.class, "z");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotorSimple.class, "rb");
        ar = hardwareMap.get(DcMotor.class, "ar");
        le = hardwareMap.get(DcMotor.class, "le");
        le2 = hardwareMap.get(DcMotor.class, "le2");

        le2.setDirection(DcMotor.Direction.FORWARD);
        le.setDirection(DcMotor.Direction.FORWARD);
        le.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        le2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    double up = 0;

    double arp  = 0;
    @Override
    public void loop() {
        double sp = le.getCurrentPosition();
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

/*
        if (gamepad1.left_bumper) {
            g = .3;
        }
        if (gamepad1.right_bumper) {
            g = -.3;
        }
        z.setPower(g);
        g = 0;

*/
        if (gamepad1.dpad_up && t == false){
            v += .05;
            t = true;
        }
        if (gamepad1.dpad_up != true){
            t = false;
        }


        if (gamepad1.dpad_down && o == false){
            v -= .05;
            o = true;
        }
        if (gamepad1.dpad_down != true){
            o = false;
        }


        if (gamepad1.dpad_left && u == false){
            k += .05;
            u = true;

        }
        if (gamepad1.dpad_left != true){
            u = false;
        }

        
        if (gamepad1.dpad_right && i == false){
            k -= .05;
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
        if (gamepad1.a){
            x.setPosition(.7);
            y.setPosition(.65);
        }
        if (gamepad1.b) {
            x.setPosition(.85);
            y.setPosition(.5);
        }
        if (gamepad2.dpad_up && sp < 4200) {
            up = 1;
        }
        else if (gamepad2.dpad_down && sp > 0) {
            up = -1;
        }
        le.setPower(up);
        le2.setPower(up);
        up = 0;
        arp = 0;
        if (gamepad1.right_bumper){
            arp = .6;
        }

        if (gamepad1.left_bumper){
            arp = -.6;
        }
        ar.setPower(arp);
        le.setPower(up);
        telemetry.addData("Status", "x position: " + v);
        telemetry.addData("Status2", "y position: " + k);
        telemetry.addData("Linear Slide", sp);
        

//whynot
    }
    @Override
    public void stop() {
    }
}
