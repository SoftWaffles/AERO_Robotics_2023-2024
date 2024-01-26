package org.firstinspires.ftc.teamcode.opmodes;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
    boolean intake_mode = false;
    boolean intake_lock = false;
    boolean outtake_mode = false;
    boolean outtake_lock = false;
    double timeMark = 0.0;

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

            /* GAMEPAD 1 */
            robot.fieldCentric(-gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x);
            teleUpdate();
            
            if(gamepad1.left_stick_button){
                robot.resetHeading();
            }
            
            //robot.lift(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.y);

            if(gamepad1.dpad_right){
                robot.turnToAngle(-90);
            } else if(gamepad1.dpad_left){
                robot.turnToAngle(90);
            } else if(gamepad1.dpad_up){
                robot.turnToAngle(180);
            } else if(gamepad1.dpad_down){
                robot.turnToAngle(180);
            }

            // claw toggle
            if(gamepad1.right_bumper && !intake_lock && !intake_mode){
                telemetry.addData("arm position: ", robot.intakeArm.getPosition());
                telemetry.update();
                if (0.1 > robot.intakeArm.getPosition() - robot.intakeArm_closed) {
                    robot.intake1.setPosition(robot.intake1_open + 0.23);
                    robot.intake2.setPosition(robot.intake2_open - 0.25);
                    telemetry.addData("pleasese ", "YYA");
                }
                else {
                    robot.intake1.setPosition(robot.intake1_open);
                    robot.intake2.setPosition(robot.intake2_open);
                }

                intake_lock = true;
                intake_mode = true;

            } else if(gamepad1.right_bumper && !intake_lock && intake_mode ){
                robot.intake1.setPosition(robot.intake1_closed);
                robot.intake2.setPosition(robot.intake2_closed);
                intake_mode = false;
                intake_lock = true;

            }else if(!gamepad1.right_bumper && intake_lock){
                intake_lock = false;
            }

            // intake arm
            if(gamepad1.y){
                robot.intakeArm.setPosition(robot.intakeArm_closed);
            }

            if(gamepad1.a){
                robot.intakeArm.setPosition(robot.intakeArm_open);
            }

            /* GAMEPAD 2 */
            // arm
            if(gamepad2.y){
                robot.arm1.setPosition(robot.arm1_open);
                robot.arm2.setPosition(robot.arm2_open);
            }

            if(gamepad2.a){
                robot.arm1.setPosition(robot.arm1_closed);
                robot.arm2.setPosition(robot.arm2_closed);
            }

            // outtake fingers
            if(gamepad2.right_bumper && !outtake_lock && !outtake_mode){
                if (0.1 > robot.arm2.getPosition() - robot.arm1_closed) {
                    robot.outtake1.setPosition(0.7);
                    robot.outtake2.setPosition(0.3);
                }
                else {
                    robot.outtake1.setPosition(robot.outtake1_open);
                    robot.outtake2.setPosition(robot.outtake2_open);
                }

                outtake_lock = true;
                outtake_mode = true;

            } else if(gamepad2.right_bumper && !outtake_lock && outtake_mode){
                robot.outtake1.setPosition(robot.outtake1_closed);
                robot.outtake2.setPosition(robot.outtake2_closed);
                outtake_mode = false;
                outtake_lock = true;

            }else if(!gamepad2.right_bumper && outtake_lock){
                outtake_lock = false;
            }


            // linear slides
            if(gamepad2.dpad_up){
                robot.lift1.setTargetPosition(robot.lift1_up);
                robot.lift2.setTargetPosition(robot.lift2_up);
                robot.lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift1.setPower(robot.MAX_POWER);
                robot.lift2.setPower(robot.MAX_POWER);
            }

            if(gamepad2.dpad_down){
                robot.lift1.setTargetPosition(robot.lift1_down);
                robot.lift2.setTargetPosition(robot.lift2_down);
                robot.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift1.setPower(robot.MAX_POWER);
                robot.lift2.setPower(robot.MAX_POWER);
            }
            robot.lift(-gamepad2.left_stick_y>0.5, gamepad2.left_stick_y>0.5, gamepad2.left_stick_button);

            telemetry.addData("lift 1 position: ", robot.lift1.getCurrentPosition());
            telemetry.addData("lift 2 position: ", robot.lift2.getCurrentPosition());
            telemetry.update();
        }
    }
    private void teleUpdate(){
        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        dashboardTelemetry.addData("Heading:",botHeading);
        dashboardTelemetry.update();
    }
}