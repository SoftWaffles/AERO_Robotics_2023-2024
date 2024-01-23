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
    public DcMotorEx lift_left = null;
    public DcMotorEx lift_right = null;

    // SERVOS - INTAKE
    public Servo intake = null;
    public Servo in_wrist = null;
    public Servo in_arm = null;

    // SERVOS - OUTTAKE
    public Servo outtake = null;
    public Servo out_wrist = null;
    public Servo out_arm = null;

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

    // SERVO POSITIONS
    public static double in_wrist_open = 0.3;
    public static double in_wrist_closed = 0.5;

    public static double in_arm_closed = 1;
    public static double in_arm_open = 0.6;

    public static double outtake_open = 0.25;
    public static double outtake_closed = 0.1;

    public static double out_arm_open = 0.20;
    public static double out_arm_closed = 0.80;

    public static double out_wrist_open = 0.45;
    public static double out_wrist_closed = 0.08;

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
        lift_left = myOpMode.hardwareMap.get(DcMotorEx.class, "lift_left");
        lift_right = myOpMode.hardwareMap.get(DcMotorEx.class, "lift_right");

        //servos
        intake = myOpMode.hardwareMap.get(Servo.class, "intake");
        in_wrist = myOpMode.hardwareMap.get(Servo.class, "in_wrist");
        in_arm = myOpMode.hardwareMap.get(Servo.class, "in_arm");
        outtake = myOpMode.hardwareMap.get(Servo.class, "outtake");
        out_arm = myOpMode.hardwareMap.get(Servo.class, "out_arm");
        out_wrist = myOpMode.hardwareMap.get(Servo.class, "out_wrist");
        drone = myOpMode.hardwareMap.get(Servo.class, "drone");

        encoderState("off");

        // BRAKES THE MOTORS
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // MOTOR POWERS
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // SERVO INITIALIZE
        outtake.setPosition(outtake_closed);
        out_arm.setPosition(out_arm_closed);
        out_wrist.setPosition(out_wrist_closed);

        in_wrist.setPosition(in_wrist_closed);
        in_arm.setPosition(in_arm_closed);


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
        if(up){
            lift_left.setTargetPosition(lift_left.getCurrentPosition()+50);
            lift_right.setTargetPosition(lift_right.getCurrentPosition()+50);

            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_left.setPower(0.5);
            lift_right.setPower(0.5);
        }
        if(down){
            lift_left.setTargetPosition(lift_left.getCurrentPosition()-50);
            lift_right.setTargetPosition(lift_right.getCurrentPosition()-50);
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_left.setPower(0.5);
            lift_right.setPower(0.5);
        }
        if(reset){
            lift_left.setTargetPosition(10);
            lift_right.setTargetPosition(10);
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_left.setPower(0.5);
            lift_right.setPower(0.5);
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