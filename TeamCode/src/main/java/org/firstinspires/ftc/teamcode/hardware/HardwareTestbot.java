package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public static double MAX_POWER = 0.6;
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

    public static double out_arm_open = 0.25;
    public static double out_arm_closed = 0.80;

    public static double out_wrist_open = 0.4;
    public static double out_wrist_closed = 0.08;

    public static double drone_release = 1;

    public static double kp = 0.01;

    public HardwareTestbot(){
    }

    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {
        myOpMode = opMode;

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );

        //motors
        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = myOpMode.hardwareMap.get(DcMotor.class, "backRight");

        //servos
        intake = myOpMode.hardwareMap.get(Servo.class, "intake");
        in_wrist = myOpMode.hardwareMap.get(Servo.class, "in_wrist");
        in_arm = myOpMode.hardwareMap.get(Servo.class, "in_arm");
        outtake = myOpMode.hardwareMap.get(Servo.class, "outtake");
        out_arm = myOpMode.hardwareMap.get(Servo.class, "out_arm");
        out_wrist = myOpMode.hardwareMap.get(Servo.class, "out_wrist");
        drone = myOpMode.hardwareMap.get(Servo.class, "drone");

        encoderState("run");

        // BRAKES THE MOTORS
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // MOTOR POWERS
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // SERVO POWERS
        /*
        intake.setPosition(0);

        in_wrist.setPosition(in_wrist_open);
        in_wrist.setPosition(in_wrist_closed);

        in_wrist.setPosition(in_arm_open);
        in_wrist.setPosition(in_arm_closed);

        in_wrist.setPosition(outtake_open);
        in_wrist.setPosition(outtake_closed);

        in_wrist.setPosition(out_arm_open);
        in_wrist.setPosition(out_arm_closed);
        */

        // SERVO INITIALIZE
        outtake.setPosition(outtake_closed);
        out_arm.setPosition(out_arm_closed);
        out_wrist.setPosition(out_wrist_closed);

        in_wrist.setPosition(in_wrist_closed);
        in_arm.setPosition(in_arm_closed);


    }
    public void resetHeading(){
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );
    }
    public void roboCentric(double forw, double side, double spin) {
        double FLPow = forw - side - spin;
        double FRPow = -forw - side + spin;
        double RLPow = forw - side - spin;
        double RRPow = -forw + side - spin;
        // normalize all motor speeds so no values exceeds 100%.
        FLPow = Range.clip(FLPow, -MAX_POWER, MAX_POWER);
        FRPow = Range.clip(FRPow, -MAX_POWER, MAX_POWER);
        RLPow = Range.clip(RLPow, -MAX_POWER, MAX_POWER);
        RRPow = Range.clip(RRPow, -MAX_POWER, MAX_POWER);
        // Set drive motor power levels.
        frontLeft.setPower(FLPow);
        frontRight.setPower(FRPow);
        backLeft.setPower(RLPow);
        backRight.setPower(RRPow);
    }

    public void fieldCentric(double y, double x, double rx){

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.min(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), MAX_POWER);
        double FLPow = (rotY + rotX + rx/2) / denominator;
        double RLPow = (-rotY - rotX + rx/2) / denominator;
        double FRPow = (rotY - rotX - rx/2) / denominator;
        double RRPow = (-rotY + rotX - rx/2) / denominator;
        // Set drive motor power levels.
        frontLeft.setPower(FLPow);
        frontRight.setPower(FRPow);
        backLeft.setPower(RLPow);
        backRight.setPower(RRPow);
    }

    public void distanceDrive(LinearOpMode opMode, double forMovement,double latMovement,double turn, double speed){
        int forwardSteps = (int)(forMovement * COUNTS_PER_INCH);
        int sideSteps = (int)(latMovement * COUNTS_PER_INCH);
        turn = ROBOT_DIAMETER_INCHES*Math.PI*(turn/180.0)*Math.PI/6.5;
        int turnSteps = (int)(turn * COUNTS_PER_INCH);

        int frontleftTargetPos   = frontLeft.getCurrentPosition() + (int)(forwardSteps + sideSteps + turnSteps);
        int frontrightTargetPos  = frontRight.getCurrentPosition() + (int)(-forwardSteps - sideSteps + turnSteps);
        int backleftTargetPos    = backLeft.getCurrentPosition() + (int)(forwardSteps - sideSteps - turnSteps);
        int backrightTargetPos   = backRight.getCurrentPosition() + (int)(-forwardSteps + sideSteps - turnSteps);

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

        while (opMode.opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnToAngle(double angle){
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = botHeading - angle;
        while(error > 10){
            fieldCentric(0, 0, error*kp);
            error = botHeading - angle;
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