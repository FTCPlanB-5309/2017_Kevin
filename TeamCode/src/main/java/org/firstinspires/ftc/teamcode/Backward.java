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

    public void run(double speed, int distance){robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int target = (int) (distance * robot.COUNTS_PER_INCH);
        robot.leftWheel.setTargetPosition(-target);
        robot.rightWheel.setTargetPosition(-target);
        // Turn On RUN_TO_POSITION
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftWheel.setPower(speed);
        robot.rightWheel.setPower(speed);
        // keep looping while we are still active, and there is time left, and both motors are running.
        while (robot.leftWheel.isBusy() && robot.rightWheel.isBusy()) {
            // Allow time for other processes to run.
            Thread.yield();
        }

        // Stop all motion;
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
    }
}

