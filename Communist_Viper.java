package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import java.util.Queue;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;



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
    private IMU imu  = null;
    private ServoImpl cr = null;
    GoBildaPinpointDriver odo;

    @Override
    public void init() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
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
        imu = hardwareMap.get(IMU.class, "imu");
        cr = hardwareMap.get(ServoImpl.class, "cr");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        

        le2.setDirection(DcMotorSimple.Direction.REVERSE);
        le.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        le.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        le2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
       str_Posit = ar.getCurrentPosition();
       cur_Posit = str_Posit;
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
    double kill = 500;


    double arp  = 0;
    double tp;
    double cp;
    double gp;
    double pcp;
    double tar_Posit_ARM;
    double cur_Posit_ARM;
    double rem_Dis_ARM;
    double prop_Cont_Power_ARM;
    double killa = 500;
    double str_Posit;
    double cur_Posit;
    double prop_SPEED = .9;
    double sck = 0;

    public void arm_code() {
        if (gamepad1.x){
            x.setPosition(v);

        }
        if (gamepad1.y){
            y.setPosition(k);
        }
        if (gamepad2.a){
            x.setPosition(.7);
            y.setPosition(.65);
        }
        if (gamepad2.b) {
            x.setPosition(.85);
            y.setPosition(.5);
        }

    }
    public void drive_code() {

        telemetry.addData("Heading", getHeading());
        double sm = 2.30 - (gamepad1.right_bumper ? 1 : 0) + (gamepad1.left_bumper ? 1 : 0);
        up = 0;
        if (gamepad1.right_stick_x > 0) {
            sm -= 0.75;

        }
        sm = 1.4;
        lp = (-gamepad1.left_stick_y / sm) + (-gamepad1.right_stick_x / sm);
        rp = (-gamepad1.left_stick_y / sm) + (gamepad1.right_stick_x / sm);

        if (gamepad2.left_bumper) {
            lb.setPower(0);
            lf.setPower(0);
            rb.setPower(0);
            rf.setPower(0);
        }else if (gamepad1.left_stick_x >= 1) {
            lb.setPower(-0.9);
            lf.setPower(0.9);
            rb.setPower(0.9);
            rf.setPower(-0.9);
        }else if (gamepad1.left_stick_x <= -1) {
            lb.setPower(0.9);
            lf.setPower(-0.9);
            rb.setPower(-0.9);
            rf.setPower(0.9);
        }else {
            lb.setPower(rp);
            lf.setPower(rp);
            rb.setPower(lp);
            rf.setPower(lp);
        }



    }

    @Override
    public void loop() {
        drive_code();
        arm_code();
        double sp = le2.getCurrentPosition();
        str_Posit = cur_Posit;
        cur_Posit = ar.getCurrentPosition();
        double posit_Diff = cur_Posit - str_Posit;
        double arm = ar.getCurrentPosition();

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
        if (gamepad2.a){
            x.setPosition(.7);
            y.setPosition(.65);
        }
        if (gamepad2.b) {
            x.setPosition(.85);
            y.setPosition(.5);
        }
        if (gamepad2.left_stick_x >= 1){
            sck += .05;
            cr.setPosition(sck);


        }
        if (gamepad2.left_stick_x <= -1){
            sck -= .05;
            cr.setPosition(sck);
        }

        if (gamepad2.right_stick_y >= 1){
            cr.setPosition(.5);
        }
        if (gamepad2.right_stick_y <= -1){
            cr.setPosition(.15);
        }


        if (gamepad2.x) {
            Linear_Preset(-5000);
        }
        else if (gamepad2.y) {
            Linear_Preset(-50);
        }
        else if (gamepad2.dpad_left) {
            Linear_Preset(-2700);
        }
        else if (gamepad2.dpad_right){
            Linear_Preset(-2700);
        }
        else{
            up = 0;
            if (gamepad2.dpad_up && sp > -6200) {
                up = 1;
            }
            else if (gamepad2.dpad_down && sp < 0) {
                up = -1;
            }


            le.setPower(up);
            le2.setPower(up);
            sp = le2.getCurrentPosition();
        }

        if (gamepad2.left_trigger > 0.1) {
            tar_Posit_ARM = 850;
            cur_Posit_ARM = ar.getCurrentPosition();
            rem_Dis_ARM = tar_Posit_ARM - cur_Posit_ARM;
            if (abs(rem_Dis_ARM) > killa){
                if (rem_Dis_ARM > 0){
                    prop_Cont_Power_ARM = -prop_SPEED;

                }
                if (rem_Dis_ARM < 0){
                    prop_Cont_Power_ARM = prop_SPEED;
                }
            } else {
                prop_Cont_Power_ARM = (-rem_Dis_ARM/killa)*prop_SPEED + posit_Diff * .005;

            }
            ar.setPower(prop_Cont_Power_ARM);

        }
        else if (gamepad2.right_trigger != 0){
            tar_Posit_ARM = 0;
            cur_Posit_ARM = ar.getCurrentPosition();
            rem_Dis_ARM = tar_Posit_ARM - cur_Posit_ARM;
            if (abs(rem_Dis_ARM) > killa){
                if (rem_Dis_ARM > 0){
                    prop_Cont_Power_ARM = -prop_SPEED;

                }
                if (rem_Dis_ARM < 0){
                    prop_Cont_Power_ARM = prop_SPEED;
                }
            } else {
                prop_Cont_Power_ARM = (-rem_Dis_ARM/killa)*prop_SPEED ;


            }
            ar.setPower(prop_Cont_Power_ARM);
        }
        else{
            arp = 0;
            if (gamepad2.right_bumper){
                arp = .3;
            }

            if (gamepad2.left_bumper){
                arp = -.3;
            }

            ar.setPower(arp);
        }

        



        telemetry.addData("Status", "x position: " + v);
        telemetry.addData("Status2", "y position: " + k);
        telemetry.addData("Linear Slide", sp);
        telemetry.addData("Arm Post: ", arm);
        telemetry.addData("Posit_Diff: ", posit_Diff);
        telemetry.addData("Rotate: ", sck);
        


    }
    public void Linear_Preset(double Target_Posit){
        tp = Target_Posit;
        cp = le2.getCurrentPosition();
        gp = tp - cp;
        if (abs(gp) > kill){
            if (gp > 0){
                pcp = -1;

            }
            if (gp < 0){
                pcp = 1;
            }
        } else {
            pcp = -gp/kill;

        }
        le.setPower(pcp);
        le2.setPower(pcp);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    @Override
    public void stop() {
    }

}
