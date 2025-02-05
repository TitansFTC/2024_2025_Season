package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.lang.Math;
import java.util.Calendar;
import java.util.Timer;

import java.util.Locale;
@Autonomous(name="Communist_Cobra_Loop_Auto", group="Robot")
public class Communist_Cobra_Loop extends LinearOpMode{
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
    private ServoImpl cr = null;
    //private IMU imu  = null;
    GoBildaPinpointDriver odo;
    double oldTime = 0;
    double arm_target;
    double positionX;
    double positionY;
    double tar_Posit_ARM;
    double cur_Posit_ARM;
    double rem_Dis_ARM;
    double prop_Cont_Power_ARM;
    double killa = 500;
    double prop_SPEED = .9;
    double str_Posit;
    double cur_Posit;
    double tar_Pos_ARM_MAIN;
    double rp;
    double lp;
    double up = 0;
    double kill = 500;
    double arp  = 0;
    double tp;
    double cp;
    double gp;
    double pcp;
    double Target_Posit;
    double tar_pos_X;
    double tar_pos_Y;
    double A;
    double B;
    double C;
    Pose2D pos;
    double rel_X;
    double rel_Y;
    double prop_PowX;
    double prop_PowY;
    //double prop_Scl = (1/(Math.sqrt(2)));
    double pow;
    double rel_tar_XM;
    double rel_tar_YM;
    double rel_tar_X;
    double rel_tar_Y;
    double cur_T;
    double srt_T;
    double tar_T;
    double rel_T;


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
        cr = hardwareMap.get(ServoImpl.class, "cr");
        //imu = hardwareMap.get(IMU.class, "imu");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        //imu.initialize(new IMU.Parameters(orientationOnRobot));
        le2.setDirection(DcMotorSimple.Direction.REVERSE);
        le.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        le.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        le2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        //START_HEADING = getHeading();
        odo.setOffsets(0, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
        waitForStart();
        odo.update();
        close();
        score();

        update_Tar(400, -5000, -249, 514, -3);
        res_T();
        while ( (gt_T() <= 1) ) {
            update();
            telemetry.update();
        }
        update_Tar(1000, -60, -249, 514, -3);
        res_T();
        while ( (gt_T() <= 2.25) ) {
            update();
            telemetry.update();
        }
        close();
        sleep(400);
        score();
        update_Tar(400,-5000, -507, 529, -5);
        res_T();
        while ((gt_T() <= 1)  ) {
            update();
            telemetry.update();
        }
        update_Tar(1000,-60, -507, 529, -5);
        res_T();
        while ((gt_T() <= 2.25)  ) {
            update();
            telemetry.update();
        }
        close();
        sleep(400);
        score();
        update_Tar(400,-5000, -565, 547, 16);
        res_T();
        while ((gt_T() <= 1)  ) {
            update();
            telemetry.update();
        }
        update_Tar(1000,-60, -565, 547, 16);
        res_T();
        while ((gt_T() <= 2.25)  ) {
            update();
            telemetry.update();
        }
        close();
        sleep(400);
        score();
        update_Tar(0, -5000, -405, 272, 132);
        res_T();
        while (gt_T() <= 1){
            update();
            telemetry.update();
        }
        le.setPower(0);
        le2.setPower(0);
        stop_drive();
        while (true){
        }
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
            rf.setPower(.7);
            rb.setPower(.7);
        }
        if (goal_right < 0) {
            rf.setPower(-.7);
            rb.setPower(-.7);
        }
        if (goal_left > 0){
            lf.setPower(.7);
            lb.setPower(.7);
        }
        if (goal_left < 0){
            lf.setPower(-.7);
            lb.setPower(-.7);
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
            le.setPower(.8);
            le2.setPower(.8);
        }
        if (up_inches < 0) {
            le.setPower(-.8);
            le2.setPower(-.8);
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
    /*public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void turn_goal (double turn_goal, boolean turn_right){
        double turn_posit = translate(START_HEADING + turn_goal);
        double cur_posit = translate(getHeading());
        if (turn_right == false){
            //left
            while (cur_posit < turn_posit){
                lf.setPower(-.5);
                rf.setPower(.5);
                lb.setPower(-.5);
                rb.setPower(.5);
                cur_posit = translate(getHeading());
            }
            stop_drive();
            return;
        }else {
            //right
            while (cur_posit > turn_posit){
                lf.setPower(.5);
                rf.setPower(-.5);
                lb.setPower(.5);
                rb.setPower(-.5);
                cur_posit = translate(getHeading());
            }
            stop_drive();
            return;
        }
    */
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
    public void update() {
        odo.update();

        tar_Posit_ARM = tar_Pos_ARM_MAIN;
        cur_Posit_ARM = ar.getCurrentPosition();
        rem_Dis_ARM = tar_Posit_ARM - cur_Posit_ARM;
        str_Posit = cur_Posit;
        cur_Posit = ar.getCurrentPosition();
        double posit_Diff = cur_Posit - str_Posit;
        if (abs(rem_Dis_ARM) > killa){
            if (rem_Dis_ARM > 0){
                prop_Cont_Power_ARM = prop_SPEED;
            }
            if (rem_Dis_ARM < 0){
                prop_Cont_Power_ARM = -prop_SPEED;
            }
        } else {
            prop_Cont_Power_ARM = (rem_Dis_ARM/killa)*prop_SPEED + posit_Diff * .005;
        }
        ar.setPower(prop_Cont_Power_ARM);
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
        pos = odo.getPosition();
        C = pos.getHeading(AngleUnit.DEGREES);
        rel_tar_X = tar_pos_X - pos.getX(DistanceUnit.MM);
        rel_tar_Y = tar_pos_Y - pos.getY(DistanceUnit.MM);
        if ((rel_tar_X != 0) || (rel_tar_Y != 0) || (tar_T != pos.getHeading(AngleUnit.DEGREES))) {
            double beta = 90;
            if (rel_tar_Y < 0) {
                beta = -90;
            }
            if (rel_tar_X != 0 ){
                beta = Math.toDegrees(Math.atan(rel_tar_Y/rel_tar_X));
            }
            if (rel_tar_X < 0){
                beta = beta - 180;
            }
            A = 90 + C - beta;
            rel_X = Math.sin(Math.toRadians(A));
            rel_Y = Math.cos(Math.toRadians(A));
            if (Math.abs(pos.getHeading(AngleUnit.DEGREES) - tar_T) >= 30) {
                if (tar_T > pos.getHeading(AngleUnit.DEGREES)){
                    rel_T = .5;
                }
                else {
                    rel_T = -.5;
                }
            }
            else {
                rel_T = ((tar_T - pos.getHeading(AngleUnit.DEGREES) )/30) * .5;
            }
            telemetry.update();
            telemetry.addData("rel_X: ", rel_X);
            telemetry.addData("rel_Y: ", rel_Y);
            telemetry.addData("Heading (pos): ", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("rel_tar_X: ", rel_tar_X);
            telemetry.addData("rel_tar_Y: ", rel_tar_Y);
            //telemetry.addData("Beta: ", beta);
            //telemetry.addData("A: ", A);
            //telemetry.addData("C: ", C);
            telemetry.addData("odoX: ", pos.getX(DistanceUnit.MM));
            telemetry.addData("odoY: ", pos.getY(DistanceUnit.MM));
            telemetry.addData("Arm: ", tar_Pos_ARM_MAIN);
            telemetry.addData("Arm_Pow; ", prop_Cont_Power_ARM);
            telemetry.update();
            double slow = 1;
            if (dist_tar() < 200){
                slow = dist_tar()/200;
            }
            double lfp = ((rel_Y + rel_X )  * slow - rel_T);
            double rfp = ((rel_Y + -rel_X )  * slow + rel_T);
            double lbp = ((rel_Y + -rel_X )  * slow - rel_T);
            double rbp = ((rel_Y + rel_X )  * slow + rel_T);
            if ((Math.abs(lfp) >= 1) || (Math.abs(rfp) >= 1) || (Math.abs(lbp) >= 1) || (Math.abs(rbp) >= 1)){
                double k = Math.max(Math.max(Math.abs(lfp), rfp), Math.max(rbp, lbp));
                lfp = lfp/k;
                rfp = rfp/k;
                rbp = rbp/k;
                lbp = lbp/k;
            }
            lf.setPower(lfp);
            rf.setPower(rfp);
            lb.setPower(lbp);
            rb.setPower(rbp);
        }


    }
    public void update_Tar(double at, double lt, double tx, double ty, double tt) {
        tar_Pos_ARM_MAIN = at;
        Target_Posit = lt;
        tar_pos_X = tx;
        tar_pos_Y  = ty;
        tar_T = tt;


    }
    public double dist_tar(){
        odo.update();
        pos = odo.getPosition();
        double distance = Math.sqrt(Math.pow((tar_pos_X - pos.getX(DistanceUnit.MM)), 2) + (Math.pow((tar_pos_Y - pos.getY(DistanceUnit.MM)), 2)));
        return(distance);
    }
    public void res_T(){
         srt_T = (double) System.currentTimeMillis()/1000.0;
         cur_T = (double) System.currentTimeMillis()/1000.0;
    }
    public double gt_T(){
        cur_T = (double) System.currentTimeMillis()/1000.0;
        return (cur_T- srt_T);
    }
    public void close(){
        x.setPosition(.7);
        y.setPosition(.65);
    }
    public void open(){
        x.setPosition(.8);
        y.setPosition(.55);
    }
    public void score(){
        update_Tar(0, -5000, -405, 272, 132);
        cr.setPosition(.5);
        res_T();
        while (gt_T() <= 2.25  ) {
            update();
            telemetry.update();
        }
        update_Tar(400, -5000, -405, 272, 132);
        res_T();

        while (gt_T() <= 1.5){
            update();
            telemetry.update();
        }
        open();
    }

}
