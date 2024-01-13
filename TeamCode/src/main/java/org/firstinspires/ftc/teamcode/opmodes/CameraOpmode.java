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

  public static double forwardDist = 0;
  public static double sideDist = 0;
  public static double speed = 0.5;
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
      sleep(100L);
      telemetry.addData("Identified", color.getSelection());
      telemetry.update();

      if(color.getSelection()==0 || color.getSelection()==1){
        // IF NONE OR LEFT
        robot.encoderState("reset");
        robot.encoderState("run");
        // distanceDrive(LinearOpMode opMode, double forMovement,double latMovement,double turn, double speed)
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // forward
        //robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // rotate left
        //robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // forward a little
        //robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // back a little
        //robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // left
        //robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // forward
        //robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // right a little
        //robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // forward
        //score();

      } else if(color.getSelection()==2){
        // IF MIDDLE
        robot.encoderState("reset");
        robot.encoderState("run");
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // forward
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // forward a little
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // back a little
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // left
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // rotate left
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // forward a little
        score();
      } else {
        // IF RIGHT
        robot.encoderState("reset");
        robot.encoderState("run");
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // forward
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // rotate right
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // forward a little
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // back
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // rotate 180
        robot.distanceDrive(this, forwardDist, sideDist, turn,speed); // right a little
        score();
      }
    }
  }

  public void score(){
    //intake down
    robot.in_wrist.setPosition(robot.in_wrist_open);
    robot.in_arm.setPosition(robot.in_arm_open);

    sleep(2000);

    // bring out
    robot.outtake.setPosition(robot.outtake_closed);
    robot.out_arm.setPosition(robot.out_arm_open);
    robot.out_wrist.setPosition(robot.out_wrist_open);

    sleep(2000);

    //drop pixels
    robot.outtake.setPosition(robot.outtake_open);

    sleep(2000);

    // reset positions
    robot.out_arm.setPosition(robot.out_arm_closed);
    robot.out_wrist.setPosition(robot.out_wrist_closed);
  }
}