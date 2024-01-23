package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    FtcDashboard dashboard =  FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    double newAngle = 0;

    @Override
    public void runOpMode() {
        robot.initDrive(this);
        robot.encoderState("reset");
        robot.encoderState("run");

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();
        waitForStart();
        //run loop while button pressed
        while (opModeIsActive() && !isStopRequested()){
            robot.fieldCentric(-gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x);

            teleUpdate();
            if(gamepad1.left_stick_button){
                robot.resetHeading();
            }
            robot.lift(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.y);

            if(gamepad1.dpad_right){
                newAngle = -90;
            } else if(gamepad1.dpad_left){
                newAngle = 90;
            } else if(gamepad1.dpad_up){
                newAngle = 180;
            } else if(gamepad1.dpad_down){
                newAngle = -180;
            }
            robot.turnToAngle(newAngle);

        }
    }
    private void teleUpdate(){
        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        dashboardTelemetry.addData("Heading:",botHeading);
        dashboardTelemetry.update();
    }
}