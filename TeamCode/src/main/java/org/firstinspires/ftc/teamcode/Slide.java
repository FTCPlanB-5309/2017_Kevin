package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide {
    RobotHardware robot;
    Telemetry telemetry;

    public Slide(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void run(double speed, int distance, int direction) throws InterruptedException{
        if(direction == robot.LEFT)
            direction = 1;
        else
            direction = -1;
        robot.centerWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thread.sleep(250);
        int target = (int) (distance * robot.COUNTS_PER_INCH)*direction;
        robot.centerWheel.setTargetPosition(target);
        robot.centerWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.centerWheel.setPower(speed*direction);
        while (robot.centerWheel.isBusy()) {
            Thread.yield();
        }

        robot.centerWheel.setPower(0);
    }
}

