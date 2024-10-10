

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Communist Spider: Main OpMode", group="Titans TeleOps")
public class Communist_Spider_Main_Op_Mode extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rf = null;
    private DcMotorSimple lf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private DcMotor le = null;
    private CRServo sa = null;
    private CRServo sb = null;
    private Servo sc = null;
    private Servo lr = null;
    private Servo rr = null;
    private Servo br = null;
    private double rs;
    private double BUCKET_UP = 0.275;
    private double BUCKET_MID = 0.33;
    private double BUCKET_DUMP = 0.41;
    //private DcMotor lt = null;
    private DcMotor le2 = null;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        rf = hardwareMap.get(DcMotor.class, "right_front");
        lf = hardwareMap.get(DcMotorSimple.class, "left_front");
        lb = hardwareMap.get(DcMotor.class, "left_back");
        rb = hardwareMap.get(DcMotor.class, "right_back");
        le = hardwareMap.get(DcMotor.class, "lift_elevator");
        sa = hardwareMap.get(CRServoImpl.class, "right_grab" );
        sb = hardwareMap.get(CRServoImpl.class, "left_grab" );
        br = hardwareMap.get(ServoImpl.class, "bucket_raise");
        //lt = hardwareMap.get(DcMotor.class, "launcher_test");
        le2 = hardwareMap.get(DcMotor.class, "lift_elevator2");

        rs = BUCKET_MID;
        br.setPosition(rs);


        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        le2.setDirection(DcMotor.Direction.FORWARD);
        le.setDirection(DcMotor.Direction.FORWARD);
        //lt.setDirection(DcMotor.Direction.FORWARD);
        le.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        le2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }
    double gr = 0;
    @Override
    public void loop() {
        double sp = le.getCurrentPosition();
        double lp;
        double rp;
        double up = 0;
        double lrp = 0;
        double rrp = 0;
        double ltt = 0;
        double up2 = 0;
        double sp2 = le2.getCurrentPosition();
        //////////////////// Driving \\\\\\\\\\\\\\\\\\\\
        // For SM Lower = Faster
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

        //////////////////// linear Slide \\\\\\\\\\\\\\\\\\\\
        /*if (gamepad2.dpad_up && sp != 5800) {
            if (sp > 5900) {
                up = -1.75;
            } else if (sp < 5700) {
                up = 1.75;
            }
        } else if (gamepad2.dpad_down && sp > 0) {
            up = -1.75;
        } else if (gamepad2.dpad_right && sp != 4200) {
            if (sp > 4300) {
                up = -1.75;
            } else if (sp < 4100) {
                up = 1.75;
            }
        } else if (gamepad2.dpad_left && sp != 2200) {
            if (sp > 2300) {
                up = -1.75;
            } else if (sp < 2100) {
                up = 1.75;
            }
        }
*/
        if (gamepad2.dpad_up && sp < 4200) {
            up = 1;
        }
        else if (gamepad2.dpad_down && sp > 0) {
            up = -1;
        }
        le.setPower(up);


       /* if (gamepad2.dpad_up && sp2 != 5800) {
            if (sp2 > 5900) {
                up2 = -1.75;
            } else if (sp2 < 5700) {
                up2 = 1.75;
            }
        } else if (gamepad2.dpad_down && sp2 > 0) {
            up2 = -1.75;
        } else if (gamepad2.dpad_right && sp2 != 4200) {
            if (sp2 > 4300) {
                up2 = -1.75;
            } else if (sp2 < 4100) {
                up2 = 1.75;
            }
        } else if (gamepad2.dpad_left && sp2 != 2200) {
            if (sp2 > 2300) {
                up2 = -1.75;
            } else if (sp2 < 2100) {
                up2 = 1.75;
            }
        }
*/
        le2.setPower(up);
        //////////////////// Grabber Servos \\\\\\\\\\\\\\\\\\\\
        if (gamepad2.right_bumper) {
            gr = -0.5;
        } else if (gamepad2.left_bumper) {
            gr = 0.5;
        } else {
            gr = 0;
        }

        sa.setPower(gr);
        sb.setPower(-gr);

        if (gamepad2.y && sp < 500) {
            // Up
            rs = BUCKET_UP;
        } else if (gamepad2.x) {
            // Mid
            rs = BUCKET_MID;
        } else if(gamepad2.a && sp > 1500) {
            // Dump
            rs = BUCKET_DUMP;
        }
        if (sp < 1500 && up < 0) {
            rs = BUCKET_MID;
        }
        br.setPosition(rs);
        /*
       if (gamepad2.x) {
           rs -= .001;
           br.setPosition(rs);
       }
       if (gamepad2.y) {
           rs += .001;
           br.setPosition(rs);
       }
        */










        ///////////////////// Random Updates \\\\\\\\\\\\\\\\\\\\
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", gamepad1.left_stick_x);
        telemetry.addData("Wat?", "launcher: "  + ltt);
        //telemetry.addData("Status", "launcher Power: "  + ltt);
        telemetry.addData("Linear Slide", sp);
        telemetry.addData("Bucket", rs);
    }

    @Override
    public void stop() {
    }
}