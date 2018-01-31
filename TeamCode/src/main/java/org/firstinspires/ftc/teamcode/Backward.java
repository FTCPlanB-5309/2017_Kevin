package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Backward {

    RobotHardware robot;
    Telemetry telemetry;

    public Backward(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void run(double speed, int distance) throws InterruptedException{
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Thread.sleep(250);
        int target = (int) (distance * robot.COUNTS_PER_INCH);
        robot.rightWheel.setTargetPosition(-target);
        robot.leftWheel.setTargetPosition(-target);
        // Turn On RUN_TO_POSITION
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightWheel.setPower(speed);
        robot.leftWheel.setPower(speed);
        // keep looping while we are still active, and there is time left, and both motors are running.
        while (robot.leftWheel.isBusy() && robot.rightWheel.isBusy()) {
            telemetry.addData("left:  ", robot.leftWheel.getCurrentPosition());
            telemetry.addData("right: ", robot.rightWheel.getCurrentPosition());
            telemetry.update();

            // Allow time for other processes to run.
            Thread.yield();
        }

        // Stop all motion;
        robot.rightWheel.setPower(0);
        robot.leftWheel.setPower(0);
    }
}

