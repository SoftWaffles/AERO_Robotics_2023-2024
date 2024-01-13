package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

  public static double LEFT_FORW_DIST = 0;
  public static double LEFT_SIDE_DIST = 0;
  public static double MID_FORW_DIST = 0;
  public static double MID_SIDE_DIST = 0;
  public static double RIGHT_FORW_DIST = 0;
  public static double RIGHT_SIDE_DIST = 0;
  public static double speed = 0.5;

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
      sleep(100L);
      telemetry.addData("Identified", color.getSelection());
      telemetry.update();

      if(color.getSelection()==0 || color.getSelection()==1){
        robot.encoderState("reset");
        robot.encoderState("run");
        robot.distanceDrive(LEFT_FORW_DIST, LEFT_SIDE_DIST, speed);
      } else if(color.getSelection()==2){
        robot.encoderState("reset");
        robot.encoderState("run");
        robot.distanceDrive(MID_FORW_DIST, MID_SIDE_DIST, speed);
        // code for if MIDDLE
      } else if(color.getSelection()==3){
        robot.encoderState("reset");
        robot.encoderState("run");
        robot.distanceDrive(RIGHT_FORW_DIST, RIGHT_SIDE_DIST, speed);
        // code for if RIGHT
      }
    }
  }
}