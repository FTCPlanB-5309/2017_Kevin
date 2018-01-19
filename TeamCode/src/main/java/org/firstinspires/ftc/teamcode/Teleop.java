package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tele-op", group = "Robot")
@SuppressWarnings("unused")

public class Teleop extends OpMode {
    RobotHardware robot = new RobotHardware(telemetry);
    float getStickValue(float joy){
        if(-joy < -robot.DEADZONE){
            return ((-joy+robot.DEADZONE)/(1-robot.DEADZONE));
        }
        else if (-joy > robot.DEADZONE){
            return ((-joy-robot.DEADZONE)/(1-robot.DEADZONE));
        }
        else {
            return(0);
        }
    }
    Relic relic = new Relic(robot, telemetry);
    Glyph glyph = new Glyph(robot, telemetry);
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        updateTelemetry(telemetry);
        robot.jewelArmMotor.setPower(0.05);

        float lx=getStickValue(gamepad1.left_stick_x);
        robot.centerWheel.setPower(lx);

        float ly=getStickValue(gamepad1.left_stick_y);
        float rx=getStickValue(gamepad1.right_stick_x);
        robot.leftWheel.setPower(ly - rx);
        robot.rightWheel.setPower(ly + rx);

        //
        robot.armMotor.setPower(-gamepad2.right_stick_y/2);
        robot.relicArm.setPower(-gamepad2.left_stick_y);

       if (gamepad2.right_bumper) {
            glyph.open();
        }
        else if (gamepad2.right_trigger > 0.4) {
            glyph.close();
        }
        else if (gamepad2.right_stick_button){
           glyph.soft();
        }

        if (gamepad2.left_bumper) {
            relic.claw(robot.OPEN);
        }
        else if (gamepad2.left_trigger > 0.4) {
            relic.claw(robot.CLOSE);
        }

        // handle extension arm
        if(gamepad2.dpad_up)
            relic.extend(1);
        if(gamepad2.dpad_down)
            relic.extend(0);
        if(!gamepad2.dpad_down && !gamepad2.dpad_up)
            relic.extend(0.5);

        // handle wrist (open close rotate
        if(gamepad2.dpad_left)
            relic.wrist(-0.01);
        if(gamepad2.dpad_right)
            relic.wrist(0.01);


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

