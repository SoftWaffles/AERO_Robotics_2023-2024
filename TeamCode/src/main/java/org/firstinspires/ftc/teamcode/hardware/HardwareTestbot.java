package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class HardwareTestbot
{

    // DEFINE OPMORE MEMBERS
    private LinearOpMode myOpMode;

    // ACCESS INSTRUMENTS OF HUB
    public IMU imu;
    Orientation angle;
    double botHeading;

    // SENSORS

    // MOTOR DECLARATIONS - MOVEMENT
    public DcMotor frontLeft   = null;
    public DcMotor frontRight  = null;
    public DcMotor backLeft   = null;
    public DcMotor backRight  = null;

    // MOTOR DECLARATIONS - SUBSYSTEMS
    public DcMotorEx lift1 = null;
    public DcMotorEx lift2 = null;

    // SERVOS
    public Servo intake1 = null;
    public Servo intake2 = null;
    public Servo intakeArm = null;

    public Servo outtake1 = null;
    public Servo outtake2 = null;
    public Servo arm1 = null;
    public Servo arm2 = null;

    public Servo drone = null;

    // MOTOR POWERS
    public static double     MAX_POWER = 0.5;
    public static double     STRAFE_GAIN = 1.3;
    public static double     COUNTS_PER_MOTOR_REV    = 537.7; // 28 for REV ;
    public static double     DRIVE_GEAR_REDUCTION    = 1.0; //   12 for REV;
    public static double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static double     ROBOT_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // SLIDES POSITIONS
    public static int lift1_up = 1000;
    public static int lift1_down = 1;

    public static int lift2_up = 1000;
    public static int lift2_down = 1;

    // SERVO POSITIONS
    public static double intake1_open = 0.2;
    public static double intake1_closed = 0.55;

    public static double intake2_open = 0.8;
    public static double intake2_closed = 0.45;

    public static double intakeArm_open = 1;
    public static double intakeArm_closed = 0.35;

    public static double outtake1_open = 0.35;
    public static double outtake1_closed = 0.1;

    public static double outtake2_open = 0.5;
    public static double outtake2_closed = 0.8;

    public static double outtake1_small = 0.4;
    public static double outtake2_small= 0.5;

    public static double arm1_open = 0.93;
    public static double arm1_closed = 0.1;

    public static double arm2_open = 0.07;
    public static double arm2_closed = 0.9;
    public static double drone_release = 1;

    public static double kp = 0.03;
    public static double errorMargin = 5;

    FtcDashboard dashboard =  FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public HardwareTestbot(){
    }

    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {
        myOpMode = opMode;

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //motors
        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = myOpMode.hardwareMap.get(DcMotor.class, "backRight");
        lift1 = myOpMode.hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = myOpMode.hardwareMap.get(DcMotorEx.class, "lift2");

        //servos
        intake1 = myOpMode.hardwareMap.get(Servo.class, "intake1");
        intake2 = myOpMode.hardwareMap.get(Servo.class, "intake2");
        intakeArm = myOpMode.hardwareMap.get(Servo.class, "intakeArm");
        outtake1 = myOpMode.hardwareMap.get(Servo.class, "outtake1");
        outtake2 = myOpMode.hardwareMap.get(Servo.class, "outtake2");
        arm1 = myOpMode.hardwareMap.get(Servo.class, "arm1");
        arm2 = myOpMode.hardwareMap.get(Servo.class, "arm2");

        encoderState("off");

        // BRAKES THE MOTORS
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        lift2.setDirection(DcMotor.Direction.REVERSE);


        // MOTOR POWERS
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // SERVO INITIALIZE *********************************************************************************************************
        outtake1.setPosition(outtake1_small);
        outtake2.setPosition(outtake2_small);
        arm1.setPosition(arm1_closed);
        arm2.setPosition(arm2_closed);



    }
    public void resetHeading(){
        imu.resetYaw();
    }
    public void roboCentric(double forw, double side, double spin) {
        double denominator = Math.max(Math.abs(forw) + Math.abs(side) + Math.abs(spin), 1);
        double FLPow = (forw - side - spin)/denominator;
        double FRPow = (forw + side + spin)/denominator;
        double BLPow = (-forw + side - spin)/denominator;
        double BRPow = (-forw - side + spin)/denominator;

        // Set drive motor power levels.
        frontLeft.setPower(FLPow*MAX_POWER);
        frontRight.setPower(FRPow*MAX_POWER);
        backLeft.setPower(BLPow*MAX_POWER);
        backRight.setPower(BRPow*MAX_POWER);
    }

    public void fieldCentric(double y, double x, double rx){

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(botHeading) - y * sin(botHeading);
        double rotY = x * sin(botHeading) + y * Math.cos(botHeading);

        rotX = rotX * STRAFE_GAIN;  // Counteract imperfect strafing


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (-rotY + rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX - rx) / denominator;
        double frontRightPower = (-rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX + rx) / denominator;

        // Set drive motor power levels.
        frontLeft.setPower(frontLeftPower*MAX_POWER);
        frontRight.setPower(frontRightPower*MAX_POWER);
        backLeft.setPower(backLeftPower*MAX_POWER);
        backRight.setPower(backRightPower*MAX_POWER);
    }

    public void distanceDrive(double forMovement,double latMovement,double turn, double speed){
        int y = (int)(forMovement * COUNTS_PER_INCH);
        int x = (int)(latMovement * COUNTS_PER_INCH);
        turn = ROBOT_DIAMETER_INCHES*Math.PI*(turn/180.0)*Math.PI/6.5;
        int rx = (int)(turn * COUNTS_PER_INCH);

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(botHeading) - y * sin(botHeading);
        double rotY = x * sin(botHeading) + y * Math.cos(botHeading);

        rotX = rotX * STRAFE_GAIN;  // Counteract imperfect strafing


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        int frontleftTargetPos =(int)(-rotY + rotX - rx);
        int backleftTargetPos = (int)(rotY - rotX - rx);
        int frontrightTargetPos = (int)(-rotY - rotX + rx);
        int backrightTargetPos = (int)(rotY + rotX + rx);

        frontLeft.setTargetPosition(frontleftTargetPos);
        frontRight.setTargetPosition(frontrightTargetPos);
        backLeft.setTargetPosition(backleftTargetPos);
        backRight.setTargetPosition(backrightTargetPos);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        while(frontLeft.isBusy() || backLeft.isBusy() || frontRight.isBusy() || backRight.isBusy()){
        }
    }

    public void turnToAngle(double angle){
        encoderState("off");
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = botHeading - angle;
        while(Math.abs(error) > errorMargin && myOpMode.opModeIsActive()){
            fieldCentric(0, 0, error*kp);
            error = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - angle;
        }
        encoderState("reset");
        encoderState("run");
    }



    public void lift(boolean up, boolean down, boolean reset){
        if(up && (!(lift1.getCurrentPosition()>1846) || !(lift2.getCurrentPosition()>1888))){
            lift1.setTargetPosition(lift1.getCurrentPosition()+50);
            lift2.setTargetPosition(lift2.getCurrentPosition()+50);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(0.5);
            lift2.setPower(0.5);
        }
        if(down && (!(lift1.getCurrentPosition()<-17) || !(lift2.getCurrentPosition()<24))){
            lift1.setTargetPosition(lift1.getCurrentPosition()-50);
            lift2.setTargetPosition(lift2.getCurrentPosition()-50);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(0.5);
            lift2.setPower(0.5);
        }
        if(reset){
            lift1.setTargetPosition(10);
            lift2.setTargetPosition(10);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(0.5);
            lift2.setPower(0.5);
        }
    }

    public void encoderState(String a){
        if(a.equals("reset")){
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }else if( a.equals("run")){
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if(a.equals("position")){
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else if(a.equals("off")){
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}