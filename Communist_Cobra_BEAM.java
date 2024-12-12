package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Autonomous(name="Communist_Cobra_Auto_BEAM", group="Robot")
public class Communist_Cobra_BEAM extends LinearOpMode{
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
    private ElapsedTime     runtime = new ElapsedTime();

    static final double TICKS_PER_INCH = 2000.0 / 44.0;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private double START_HEADING;

    private double rs;
    private IMU imu  = null;

    @Override
    public void runOpMode() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
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
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        le2.setDirection(DcMotorSimple.Direction.REVERSE);
        le.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        le.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        le2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        START_HEADING = getHeading();



        waitForStart();
        telemetry.addData("Heading", START_HEADING);
        double power = 0;
        x.setPosition(.7);
        y.setPosition(.65);
        drive_distance(15,15);
        sleep(1000);
        linear_distance(2000);
        ar.setPower(-.15);
        sleep(500);
        ar.setPower(0);

        x.setPosition(.7);
        y.setPosition(.65);
        sleep(5000);



    }
    public void drive_distance(double left_inches, double right_inches) {

        double goal_left = left_inches * TICKS_PER_INCH;
        double goal_right = right_inches * TICKS_PER_INCH;


        double rp;
        double lp;
        double rpt;
        double lpt;

        rp = rf.getCurrentPosition();
        lp = lb.getCurrentPosition();


        if (goal_right > 0) {
            rf.setPower(.5);
            rb.setPower(.5);
        }
        if (goal_right < 0) {
            rf.setPower(-.5);
            rb.setPower(-.5);
        }
        if (goal_left > 0){
            lf.setPower(.5);
            lb.setPower(.5);
        }
        if (goal_left < 0){
            lf.setPower(-.5);
            lb.setPower(-.5);
        }


        rpt = rp + goal_right;
        lpt = lp + goal_left;

        while ((goal_right > 0 && rp < rpt) || (goal_right < 0 && rp > rpt)
                || (goal_left > 0 && lp < lpt) || (goal_left < 0 && lp > lpt)) {

            sleep(50);
            rp = rf.getCurrentPosition();
            lp = lb.getCurrentPosition();
            if ((goal_right > 0 && rp > rpt) || (goal_right < 0 && rp < rpt)) {
                rf.setPower(0);
                rb.setPower(0);
            }
            if ((goal_left > 0 && lp >  lpt) || (goal_left < 0 && lp < lpt)) {
                lf.setPower(0);
                lb.setPower(0);
            }
        }

        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }
    public void linear_distance (double up_inches) {

        double up = abs(le2.getCurrentPosition());
        telemetry.addData("Lin: ", up);


        double goal_up = up_inches + up;
        telemetry.addData("Line: ", goal_up);





        if (up_inches > 0) {
            le.setPower(.5);
            le2.setPower(.5);
        }
        if (up_inches < 0) {
            le.setPower(-.5);
            le2.setPower(-.5);
        }





        while ((up_inches > 0 && up < goal_up) || (up_inches < 0 && up > goal_up)
        ) {

            sleep(50);
            up = abs(le2.getCurrentPosition());
            telemetry.addData("Linear:", up);



        }

        le.setPower(0);
        le2.setPower(0);

    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void turn_goal (double turn_goal, boolean turn_right){
        double turn_posit = translate(START_HEADING + turn_goal);
        double cur_posit = translate(getHeading());
        if (turn_right == false){
            //left
            while (cur_posit < turn_posit){
                lf.setPower(-.3);
                rf.setPower(.3);
                lb.setPower(-.3);
                rb.setPower(.3);
                cur_posit = translate(getHeading());

            }
            stop_drive();
            return;
        }else {
            //right
            while (cur_posit > turn_posit){
                lf.setPower(.3);
                rf.setPower(-.3);
                lb.setPower(.3);
                rb.setPower(-.3);
                cur_posit = translate(getHeading());
            }
            stop_drive();
            return;
        }
    }
    public double translate (double number){
        while (number < -180) {
            number += 360;

        }
        while (number >= 180) {
            number -= 360;
        }
        return number;
    }
    public void stop_drive() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        return;
    }

}
