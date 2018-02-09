package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Tele-op", group = "Robot")
@SuppressWarnings("unused")

public class SoloTeleop extends OpMode {

    //Object constructors.
    RobotHardware robot = new RobotHardware(telemetry);
    Jewel jewel = new Jewel(robot, telemetry);
    Relic relic = new Relic(robot, telemetry);
    Glyph glyph = new Glyph(robot, telemetry);
    //Calculating for custom deadzone
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

    @Override
    public void init() {
        robot.initTeleop(hardwareMap);
    }

    @Override
    public void loop() {
        //Preventing the jewel arm from falling during.
        jewel.moveArm(0);
        //Primary drive method.
        float lx = getStickValue(gamepad1.left_stick_x);
        robot.centerWheel.setPower(lx);

        float ly = getStickValue(gamepad1.left_stick_y);
        float rx = getStickValue(gamepad1.right_stick_x);P
        }

        //Arm controls.
        robot.armMotor.setPower(-gamepad1.right_stick_y / 2);
        if (gamepad1.a)
            robot.relicArm.setPower(-0.5);
        if (gamepad1.y)

        //Controlling the glyph grabbers. Soft open is for releasing while in the cryptobox.
        if (gamepad1.right_bumper)
            glyph.open();
        if (gamepad1.right_trigger > 0.4)
            glyph.close();
        if (gamepad1.right_stick_button)
            glyph.soft();

        //Relic claw controls.
        if (gamepad1.left_bumper)
            relic.claw(robot.OPEN);
        if (gamepad1.left_trigger > 0.4)
            relic.claw(robot.CLOSE);
        if (gamepad1.dpad_left)
            relic.wrist(-0.01);
        if (gamepad1.dpad_right)
            relic.wrist(0.01);

        //Relic extention controls.
        if (gamepad1.dpad_up)
            relic.extend(1);
        if (gamepad1.dpad_down)
            relic.extend(0);
        if (!gamepad1.dpad_down && !gamepad1.dpad_up)
            relic.extend(0.5);

        //Telemetry.
        telemetry.addData("Arm Encoder", robot.armMotor.getCurrentPosition());telemetry.addData("Left Wheel Power", (ly - rx));

        telemetry.addData("Right Wheel Power", (ly + rx));
        telemetry.update();
    }
    @Override
    public void stop() {
        robot.stopDrive();
    }
}

