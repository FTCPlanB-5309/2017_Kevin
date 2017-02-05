package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Tele-op", group="Robot")
@SuppressWarnings("unused")
public class Teleop  extends OpMode {
    RobotHardware robot       = new RobotHardware();
    @Override
    public void init() {
        robot.init(hardwareMap);
    }
    @Override
    public void loop() {
        updateTelemetry(telemetry);
        //Standard drive system
        if(!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_right && !gamepad1.dpad_left) {
            robot.leftDrive(-gamepad1.left_stick_y);
            robot.rightDrive(-gamepad1.right_stick_y);
        }
        //Slowed drive system
        if(gamepad1.dpad_up){
            robot.rightWheel.setPower(0.25);
            robot.leftWheel.setPower(0.25);
        }
        if(gamepad1.dpad_down){
            robot.rightWheel.setPower(-0.25);
            robot.leftWheel.setPower(-0.25);
        }
        if(gamepad1.dpad_left){
            robot.rightWheel.setPower(0.35);
            robot.leftWheel.setPower(-0.35);
        }
        if(gamepad1.dpad_right){
            robot.rightWheel.setPower(-0.35);
            robot.leftWheel.setPower(0.35);
        }

        //Lift
        robot.liftMotor1.setPower(-gamepad2.right_stick_y);
        robot.liftMotor2.setPower(-gamepad2.right_stick_y);

        //Beacon finger
        if(gamepad2.right_trigger > .8)
            robot.rightFingerIn();
        if(!gamepad2.right_bumper && gamepad2.right_trigger < .8)
            robot.rightFinger.setPosition(0.5);
        if(gamepad2.right_bumper)
            robot.rightFingerOut();

        //Shooter wheel-Toggles
        if(gamepad2.left_bumper)
            robot.shooterSpeed();

        //Sweeper control
        if(gamepad2.x)
            robot.sweeper1.setPower(100);
        if(gamepad2.b)
            robot.sweeper1.setPower(-100);
        if(gamepad2.a) {
            robot.sweeper1.setPower(0);
            robot.sweeper2.setPower(0);
        }
        if(gamepad2.y)
            robot.sweeper2.setPower(100);

        //Reset encoders for testing purposes
        if(gamepad1.a){
            robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //Telemetry
        telemetry.addData("Shooter Encoders L/R", robot.leftShooter.getCurrentPosition() + "/" +robot.rightShooter.getCurrentPosition());
        telemetry.addData("Range Sensor 1", robot.rangeRead());
        telemetry.addData("Range Sensor 2", robot.rangeRead2());
        telemetry.addData("Left Encoder", robot.leftWheel.getCurrentPosition());
        telemetry.addData("Right Encoder", robot.rightWheel.getCurrentPosition());
        telemetry.update();

     // Code to run ONCE after the driver hits STOP

    }
    @Override
    public void stop(){
        robot.stopDrive();
    }
}

