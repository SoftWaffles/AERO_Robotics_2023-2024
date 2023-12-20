package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.CustomProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
@Autonomous(name="VisionPortal_Test", group="Linear OpMode")
public class CameraOpmode extends LinearOpMode {
  private CustomProcessor test_proc;
  private VisionPortal visionPortal;

  private ElapsedTime runtime;

  @Override
  public void runOpMode() throws InterruptedException {
    test_proc = new CustomProcessor();
    visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), test_proc);

    waitForStart();
    runtime.reset();
    visionPortal.stopStreaming();
    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      telemetry.addData("Identified", test_proc.getSelection());
    }
  }
}