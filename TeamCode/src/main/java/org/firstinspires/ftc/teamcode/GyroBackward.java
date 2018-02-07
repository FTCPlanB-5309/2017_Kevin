package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by user on 2/6/2018.
 */

public class GyroBackward {
    RobotHardware robot;
    Telemetry telemetry;

    public GyroBackward(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void distance (double inches) {
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftWheel.setPower(-robot.PLATFORM_SPEED);
        robot.rightWheel.setPower(-robot.PLATFORM_SPEED);
        int heading = robot.gyroSensor.getHeading();
        double offset = 0;
        int finalDistance = (int) -(inches * robot.COUNTS_PER_INCH);
        while(robot.leftWheel.getCurrentPosition()  >=  finalDistance){
            heading = robot.gyroSensor.getHeading();
            if (heading == 0) {
                robot.leftWheel.setPower(-robot.PLATFORM_SPEED);
                robot.rightWheel.setPower(-robot.PLATFORM_SPEED);
            }
            if (heading > 270) {
                offset = robot.MOVEMENT_INCREMENT * (360 - heading);
                robot.leftWheel.setPower(Range.clip(-robot.PLATFORM_SPEED - offset, -0.3, 0));
                robot.rightWheel.setPower(Range.clip(-robot.PLATFORM_SPEED + offset, -0.3, 0));
            }
            if (heading < 90) {
                offset = robot.MOVEMENT_INCREMENT * heading;
                robot.leftWheel.setPower(Range.clip(-robot.PLATFORM_SPEED + offset, -0.3, 0));
                robot.rightWheel.setPower(Range.clip(-robot.PLATFORM_SPEED - offset, -0.3, 0));
            }
            telemetry.addData("offset", offset);
            telemetry.addData("heading", heading);
            telemetry.addData("left", robot.leftWheel.getPower());
            telemetry.addData("right", robot.rightWheel.getPower());
            telemetry.addData("position", robot.leftWheel.getCurrentPosition());
            telemetry.update();
        }
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
