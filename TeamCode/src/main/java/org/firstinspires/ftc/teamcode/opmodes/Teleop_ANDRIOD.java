package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.HardwareTestbot;

@TeleOp(name="DRIVE-MODE-ANDRIOD", group="testbot")
//@Disabled
public class Teleop_ANDRIOD extends LinearOpMode {

    /* Declare OpMode members. */
    ElapsedTime runtime = new ElapsedTime();
    HardwareTestbot robot = new HardwareTestbot();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        robot.initDrive(this);
        robot.encoderState("reset");
        robot.encoderState("run");
        boolean intakeToggle = false;

        robot.outtake.setPosition(robot.outtake_open);

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();
        waitForStart();
        //run loop while button pressed
        while (opModeIsActive() && !isStopRequested()){
            //robot.roboCentric(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            robot.fieldCentric(-gamepad1.left_stick_y,gamepad1.left_stick_x,-gamepad1.right_stick_x);
            teleUpdate();



            // SERVOS INTAKE
            if(gamepad2.a){
                robot.in_wrist.setPosition(robot.in_wrist_open);
                robot.in_arm.setPosition(robot.in_arm_open);
            }

            if(gamepad2.dpad_down){
                robot.in_arm.setPosition(robot.in_arm.getPosition()-0.001);
            } else if(gamepad2.dpad_up){
                robot.in_arm.setPosition(robot.in_arm.getPosition()+0.001);
            }

            if(gamepad2.b) {
                robot.in_wrist.setPosition(robot.in_wrist_closed+0.2);
                robot.in_arm.setPosition(robot.in_arm_closed);
            }

            if(gamepad2.y){
                robot.in_wrist.setPosition(robot.in_wrist_closed);
            }

            if(gamepad2.left_trigger > 0.5) {
                robot.intake.setPosition(0.8);
            } else{
                robot.intake.setPosition(0.5);
            }

            // SERVOS OUTTAKE
            if(gamepad1.dpad_up){
                robot.outtake.setPosition(robot.outtake_closed);
                robot.out_arm.setPosition(robot.out_arm_open);
                robot.out_wrist.setPosition(robot.out_wrist_open);
            }

            if(gamepad1.dpad_down){
                robot.outtake.setPosition(robot.outtake_open);
                robot.out_arm.setPosition(robot.out_arm_closed);
                robot.out_wrist.setPosition(robot.out_wrist_closed);
            }

            if(gamepad1.right_bumper){
                robot.outtake.setPosition(robot.outtake_open);
            }

            if(gamepad1.y){
                robot.drone.setPosition(robot.drone_release);
            }
            if(gamepad1.left_stick_button){
                robot.resetHeading();
            }
        }
    }
    private void teleUpdate(){
        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("Heading:",botHeading);
        telemetry.addData("beatutitfiul wrist pos:", robot.out_wrist.getPosition());
        telemetry.update();
    }
}