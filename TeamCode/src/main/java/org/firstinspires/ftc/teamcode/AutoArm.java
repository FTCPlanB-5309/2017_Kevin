package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoArm {

    RobotHardware robot;
    Telemetry telemetry;

    public AutoArm(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void run(int pos){
        int target = pos - robot.armPosition;

        robot.armMotor.setTargetPosition(target);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(0.3);

        while (robot.armMotor.isBusy())
            Thread.yield();

        robot.armMotor.setPower(0);

        robot.armPosition = pos;
    }
}

