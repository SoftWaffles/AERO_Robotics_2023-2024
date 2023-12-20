package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.CustomProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
@Autonomous()
public class CameraOpmode extends OpMode {
  private CustomProcessor test_proc;
  private VisionPortal visionPortal;

  @Override
  public void init() {
    test_proc = new CustomProcessor();
    visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), test_proc);
  }

  @Override
  public void init_loop() {
  }

  @Override
  public void start() {
    visionPortal.stopStreaming();
  }

  @Override
  public void loop() {
    telemetry.addData("Identified", test_proc.getSelection());
  }
}