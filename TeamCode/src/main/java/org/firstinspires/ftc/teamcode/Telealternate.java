package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tele-alternate", group = "Robot")
@SuppressWarnings("unused")
public class Telealternate extends OpMode {
    RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        updateTelemetry(telemetry);
        //Standard drive system
        if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_right && !gamepad1.dpad_left) {
            robot.leftDrive(-gamepad1.left_stick_y);
            robot.rightDrive(-gamepad1.right_stick_y);
        }


        //Lift
        if(gamepad1.dpad_up) {
            robot.liftMotor1.setPower(0.5);
            robot.liftMotor2.setPower(0.5);
        }
        else if(gamepad1.dpad_down){
            robot.liftMotor1.setPower(-0.5);
            robot.liftMotor2.setPower(-0.5);
        }
        else{
            robot.liftMotor1.setPower(0);
            robot.liftMotor2.setPower(0);
        }
        //Beacon finger
        if (gamepad1.right_trigger > .8)
            robot.rightFingerIn();
        if (!gamepad1.right_bumper && gamepad1.right_trigger < .8)
            robot.rightFinger.setPosition(0.5);
        if (gamepad1.right_bumper)
            robot.rightFingerOut();

        //Shooter wheel-Toggles
        if (gamepad1.left_bumper) {
            robot.shooterSpeed();
        }
        else
        {
            robot.rightShooter.setPower(0);
            robot.leftShooter.setPower(0);
        }

        //Sweeper control
        if (gamepad1.x)
            robot.sweeper1.setPower(1);
        if (gamepad1.b)
            robot.sweeper1.setPower(-1);
        if (gamepad1.a) {
            robot.sweeper1.setPower(0);
            robot.sweeper2.setPower(0);
        }
        if (gamepad1.y)
            robot.sweeper2.setPower(1);
        else{
            robot.sweeper2.setPower(0);
        }

        //Telemetry
        telemetry.addData("Shooter Encoders L/R", robot.leftShooter.getCurrentPosition() + "/" + robot.rightShooter.getCurrentPosition());
        telemetry.addData("Range Sensor 1", robot.rangeRead());
        telemetry.addData("Range Sensor 2", robot.rangeRead2());
        telemetry.addData("Left Encoder", robot.leftWheel.getCurrentPosition());
        telemetry.addData("Right Encoder", robot.rightWheel.getCurrentPosition());
        telemetry.update();

        // Code to run ONCE after the driver hits STOP

    }

    @Override
    public void stop() {
        robot.stopDrive();
    }
}

