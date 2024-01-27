package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.HardwareTestbot;
import org.firstinspires.ftc.teamcode.vision.CustomProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
@Config
@Autonomous(name="VisionPortal_Test", group="Linear OpMode")
public class CameraOpmode extends LinearOpMode {
  private CustomProcessor color;
  private HardwareTestbot robot;
  private VisionPortal visionPortal;

  private ElapsedTime runtime = new ElapsedTime();

  public static double forwardDist = 0;
  public static double sideDist = 0;
  public static double time = 2000;
  public static double turn = 0;



  @Override
  public void runOpMode() throws InterruptedException {
    robot = new HardwareTestbot();
    robot.initDrive(this);

    color = new CustomProcessor();
    visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), color);


    waitForStart();
    runtime.reset();
    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      sleep(1000);
      robot.imu.resetYaw();
      telemetry.addData("Identified", color.getSelection());
      telemetry.update();

      if(color.getSelection()==0 || color.getSelection()==1){
        // IF NONE OR LEFT
        robot.encoderState("reset");
        robot.encoderState("run");
        // move forward and turn 90
        robot.distanceDrive(5,0,0,0.5);
        robot.turnToAngle(90);
        // move back
        robot.distanceDrive(-5,0,0,0.5);
        // move left and turn
        robot.distanceDrive(-5,0,0,0.5);

        score();

      } else if(color.getSelection()==2){
        // IF MIDDLE
        robot.encoderState("reset");
        robot.encoderState("run");
        robot.distanceDrive(10,0,0,0.5);
        score();

      } else {
        // IF RIGHT
        robot.encoderState("reset");
        robot.encoderState("run");
        robot.distanceDrive(5,0,0,0.5);
        robot.turnToAngle(-90);
        robot.distanceDrive(5,0,0,0.5);

        score();

      }
      sleep(1000);
      break;
    }
  }

  public void score(){
  }
  public void drive_by_time(double forw, double side, double spin, long time){
    double FLPow = forw + side + spin;
    double FRPow = +forw - side - spin;

    double RLPow = -forw - side + spin;
    double RRPow = -forw + side - spin;
    // normalize all motor speeds so no values exceeds 100%.
    FLPow = Range.clip(FLPow, -0.5, 0.5);
    FRPow = Range.clip(FRPow, -0.5, 0.5);
    RLPow = Range.clip(RLPow, -0.5, 0.5);
    RRPow = Range.clip(RRPow, -0.5, 0.5);
    // Set drive motor power levels.
    robot.frontLeft.setPower(FLPow);
    robot.frontRight.setPower(FRPow);
    robot.backLeft.setPower(RLPow);
    robot.backRight.setPower(RRPow);
    runtime.reset();
    while(runtime.time()<time){
    }
    robot.frontLeft.setPower(0);
    robot.frontRight.setPower(0);
    robot.backLeft.setPower(0);
    robot.backRight.setPower(0);
  }
}