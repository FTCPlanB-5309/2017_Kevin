package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by user on 12/28/2017.
 */

public class ColorSensorSlide {
    RobotHardware robot;
    Telemetry telemetry;

    public ColorSensorSlide (RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void findColumn (int quadrant, RelicRecoveryVuMark column, int slideDirection) {
        int direction;
        if (slideDirection == robot.RIGHT)
            direction = -1;
        else
            direction = 1;
        robot.centerWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int target = (int) (36 * robot.COUNTS_PER_INCH) * direction;
        robot.centerWheel.setTargetPosition(target);
        robot.centerWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.centerWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.centerWheel.setPower(-0.2);

        while (robot.centerWheel.isBusy() &&( robot.FloorColorSensor.blue() <= 1 || robot.FloorColorSensor.red() <= 1)) {
            telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
            telemetry.addData("Red value: ", "%d", robot.FloorColorSensor.red());
            telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
            telemetry.update();
        }
        while (robot.centerWheel.isBusy() &&( robot.FloorColorSensor.blue() > 1|| robot.FloorColorSensor.red() <= 1)) {
            telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
            telemetry.addData("Red value: ", "%d", robot.FloorColorSensor.red());
            telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
            telemetry.update();
        }
        int line1 = robot.centerWheel.getCurrentPosition();
        while (robot.centerWheel.isBusy() &&( robot.FloorColorSensor.blue() <= 1|| robot.FloorColorSensor.red() <= 1)) {
            telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
            telemetry.addData("Red value: ", "%d", robot.FloorColorSensor.red());
            telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
            telemetry.update();
        }
        int line2 = robot.centerWheel.getCurrentPosition();

        //Going to the center of the safe zone
        int safezonecenter = (line1 + line2) / 2;
        if (column == RelicRecoveryVuMark.CENTER) {
            target = (int) (safezonecenter + robot.centerArray[quadrant] * robot.COUNTS_PER_INCH);
            robot.centerWheel.setTargetPosition(target);
        } else if (column == RelicRecoveryVuMark.RIGHT) {
            target = (int) (safezonecenter - robot.rightArray[quadrant] * robot.COUNTS_PER_INCH);
            robot.centerWheel.setTargetPosition(target);
        } else {
            target = (int) (safezonecenter + robot.leftArray[quadrant] * robot.COUNTS_PER_INCH);
            robot.centerWheel.setTargetPosition(target);
        }

        robot.centerWheel.setTargetPosition(target);
        while (robot.centerWheel.isBusy()) {
            telemetry.addData("Blue value: ", "%d", robot.FloorColorSensor.blue());
            telemetry.addData("Red value: ", "%d", robot.FloorColorSensor.red());
            telemetry.addData("Center wheel value: ", "%d", robot.centerWheel.getCurrentPosition());
            telemetry.update();
        }
        robot.centerWheel.setPower(0);
    }
}
