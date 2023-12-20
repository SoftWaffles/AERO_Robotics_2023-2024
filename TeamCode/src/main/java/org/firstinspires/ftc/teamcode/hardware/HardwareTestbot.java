package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class HardwareTestbot
{

    //define opmode members
    private LinearOpMode myOpMode;
    //access instruments of Hub
    IMU imu;
    Orientation angle;
    //sensors
    // motor declarations
    public DcMotor FLeft   = null;
    public DcMotor FRight  = null;
    public DcMotor RLeft   = null;
    public DcMotor RRight  = null;
    // motor for lift

    //motor powers
    public static double MAX_POWER = 1;
    public static double     COUNTS_PER_MOTOR_REV    = 537.7; // 28 for REV ;
    public static double     DRIVE_GEAR_REDUCTION    = 1.0; //   12 for REV;
    public static double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


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
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );

        FLeft = myOpMode.hardwareMap.get(DcMotor.class, "FLeft");
        FRight = myOpMode.hardwareMap.get(DcMotor.class, "FRight");
        RLeft = myOpMode.hardwareMap.get(DcMotor.class, "RLeft");
        RRight = myOpMode.hardwareMap.get(DcMotor.class, "RRight");


        encoderState("run");
        //Brakes the Motors
        FLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLeft.setPower(0);
        FRight.setPower(0);
        RLeft.setPower(0);
        RRight.setPower(0);
    }
    public void roboCentric(double forw, double side, double spin) {
        double FLPow = -forw + side + spin;
        double FRPow = forw + side + spin;
        double RLPow = -forw - side + spin;
        double RRPow = forw - side + spin;
        // normalize all motor speeds so no values exceeds 100%.
        FLPow = Range.clip(FLPow, -MAX_POWER, MAX_POWER);
        FRPow = Range.clip(FRPow, -MAX_POWER, MAX_POWER);
        RLPow = Range.clip(RLPow, -MAX_POWER, MAX_POWER);
        RRPow = Range.clip(RRPow, -MAX_POWER, MAX_POWER);
        // Set drive motor power levels.
        FLeft.setPower(FLPow);
        FRight.setPower(FRPow);
        RLeft.setPower(RLPow);
        RRight.setPower(RRPow);
    }
    public void fieldCentric(double y, double x, double rx){

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), MAX_POWER);
        double FLPow = (rotY + rotX + rx) / denominator;
        double RLPow = (rotY - rotX + rx) / denominator;
        double FRPow = (rotY - rotX - rx) / denominator;
        double RRPow = (rotY + rotX - rx) / denominator;
        // Set drive motor power levels.
        FLeft.setPower(FLPow);
        FRight.setPower(FRPow);
        RLeft.setPower(RLPow);
        RRight.setPower(RRPow);
    }
    public void distanceDrive(double forw_inches, double side_inches, double speed){
        int forw_tick = (int)(forw_inches * COUNTS_PER_INCH);
        int side_tick = (int)(side_inches * COUNTS_PER_INCH);

        int FLTic = FLeft.getCurrentPosition()-forw_tick + side_tick;
        int FRTic = FRight.getCurrentPosition()+forw_tick + side_tick;
        int RLTic = RLeft.getCurrentPosition()-forw_tick - side_tick;
        int RRTic = RRight.getCurrentPosition()+forw_tick - side_tick;

        FLeft.setTargetPosition(FLTic);
        FRight.setTargetPosition(FRTic);
        RLeft.setTargetPosition(RLTic);
        RRight.setTargetPosition(RRTic);

        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLeft.setPower(speed);
        FRight.setPower(speed);
        RLeft.setPower(speed);
        RRight.setPower(speed);
        while (RLeft.isBusy() && FLeft.isBusy() && FRight.isBusy() && RRight.isBusy() && myOpMode.opModeIsActive()){
        }
        FLeft.setPower(0);
        FRight.setPower(0);
        RLeft.setPower(0);
        RRight.setPower(0);
    }
    public void encoderState(String a){
        if(a.equals("reset")){
            FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }else if( a.equals("run")){
            FLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if(a.equals("position")){
            FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else if(a.equals("off")){
            FLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}