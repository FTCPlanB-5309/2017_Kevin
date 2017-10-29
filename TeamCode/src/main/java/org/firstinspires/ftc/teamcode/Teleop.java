package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Tele-op", group = "Robot")
@SuppressWarnings("unused")
public class Teleop extends OpMode {
    RobotHardware robot = new RobotHardware(telemetry);

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        updateTelemetry(telemetry);
        //Standard drive system

        if(gamepad1.left_stick_y < -.25 || gamepad1.left_stick_y > .25){
            robot.leftDrive(gamepad1.left_stick_y);
            robot.rightDrive(gamepad1.left_stick_y);
        }

        if(gamepad1.left_stick_x < -.25 || gamepad1.left_stick_x > .25)
            robot.centerWheel.setPower(-gamepad1.left_stick_x);

        if(gamepad1.left_stick_x > -.25 && gamepad1.left_stick_x < .25)
            robot.centerWheel.setPower(0);

        if(gamepad1.right_stick_x < -.25 || gamepad1.right_stick_x > 0.25){
            robot.leftDrive(-gamepad1.right_stick_x);
            robot.rightDrive(gamepad1.right_stick_x);
        }

        if(gamepad1.right_stick_x < .25 && gamepad1.right_stick_x > -.25 && gamepad1.left_stick_y > -.25 && gamepad1.left_stick_y < .25){
            robot.leftDrive(0);
            robot.rightDrive(0);
        }

        robot.armMotor.setPower(-gamepad2.right_stick_y/2);
        robot.relicArm.setPower(-gamepad2.left_stick_y/2);

       if (gamepad2.right_bumper) {
            robot.leftClaw.setPosition(robot.LEFT_CLAW_OPEN);
            robot.rightClaw.setPosition(robot.RIGHT_CLAW_OPEN);
        }
        else if (gamepad2.right_trigger > 0.4) {
            robot.leftClaw.setPosition(robot.LEFT_CLAW_CLOSED);
            robot.rightClaw.setPosition(robot.RIGHT_CLAW_CLOSED);
        }
        if (gamepad2.left_bumper) {
            robot.leftRelic.setPosition(robot.LEFT_RELIC_OPEN);
            robot.rightRelic.setPosition(robot.RIGHT_RELIC_OPEN);
        }
        else if (gamepad2.right_trigger > 0.4) {
            robot.leftRelic.setPosition(robot.LEFT_RELIC_CLOSED);
            robot.rightRelic.setPosition(robot.RIGHT_RELIC_CLOSED);
        }
        if(gamepad2.dpad_up)
            robot.extensionRelic.setPosition(robot.extensionRelic.getPosition()+0.01);
        if(gamepad2.dpad_down)
            robot.extensionRelic.setPosition(robot.extensionRelic.getPosition()-0.01);
        if(gamepad2.dpad_left)
            robot.wristRelic.setPosition(robot.wristRelic.getPosition()-0.01);
        if(gamepad2.dpad_right)
            robot.wristRelic.setPosition(robot.wristRelic.getPosition()+0.01);


        if (gamepad1.a) {
            robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.centerWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.centerWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Left Encoder", robot.leftWheel.getCurrentPosition());
        telemetry.addData("Right Encoder", robot.rightWheel.getCurrentPosition());
        telemetry.addData("Center Encoder", robot.centerWheel.getCurrentPosition());telemetry.addData("Arm Encoder", robot.armMotor.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void stop() {
        robot.stopDrive();
    }
}

