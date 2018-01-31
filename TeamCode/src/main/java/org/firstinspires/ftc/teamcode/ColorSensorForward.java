package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by user on 12/28/2017.
 */

public class ColorSensorForward {
    RobotHardware robot;
    Telemetry telemetry;
    Gyro gyro = new Gyro(robot, telemetry);

    public ColorSensorForward(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void findColumn (int leftOffset, int centerOffset, int rightOffset, RelicRecoveryVuMark column) {

        robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int target = (int) (36 * robot.COUNTS_PER_INCH);
        robot.leftWheel.setTargetPosition(target);
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftWheel.setPower(0.2);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightWheel.setTargetPosition(target);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightWheel.setPower(0.2);

        while (robot.leftWheel.isBusy() && robot.FloorColorSensor.blue() < 1) {
            telemetry.addData("blue value: ", "%d", robot.FloorColorSensor.blue());
            telemetry.addData("Left wheel value: ", "%d", robot.leftWheel.getCurrentPosition());
            telemetry.update();
        }
        while (robot.leftWheel.isBusy() && robot.FloorColorSensor.blue() >= 1) {

            telemetry.addData("blue value: ", "%d", robot.FloorColorSensor.blue());
            telemetry.addData("Left wheel value: ", "%d", robot.leftWheel.getCurrentPosition());
            telemetry.update();
        }
        int line1 = robot.leftWheel.getCurrentPosition();
        while (robot.leftWheel.isBusy() &&(robot.FloorColorSensor.blue() < 1)) {

            telemetry.addData("blue value: ", "%d", robot.FloorColorSensor.blue());
            telemetry.addData("Left wheel value: ", "%d", robot.leftWheel.getCurrentPosition());
            telemetry.update();
        }
        int line2 = robot.leftWheel.getCurrentPosition();

        int safezonecenter = (line1 + line2) / 2;
        if (column == RelicRecoveryVuMark.CENTER) {
            target = safezonecenter+centerOffset;
            robot.leftWheel.setTargetPosition(target);
            robot.rightWheel.setTargetPosition(target);
        }
        else if (column == RelicRecoveryVuMark.RIGHT) {
            target = safezonecenter + rightOffset;
            robot.leftWheel.setTargetPosition(target);
            robot.rightWheel.setTargetPosition(target);
        }
        else {
            target = safezonecenter + leftOffset;
            robot.leftWheel.setTargetPosition(target);
            robot.rightWheel.setTargetPosition(target);
        }

        robot.leftWheel.setTargetPosition(target);
        robot.rightWheel.setTargetPosition(target);
        while (robot.leftWheel.isBusy()) {
            telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
            telemetry.addData("blue value: ", "%d", robot.FloorColorSensor.blue());
            telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
            telemetry.update();
        }
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
    }
}
