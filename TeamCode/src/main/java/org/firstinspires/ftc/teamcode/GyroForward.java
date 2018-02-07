package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by user on 12/28/2017.
 */

public class GyroForward {
    RobotHardware robot;
    Telemetry telemetry;

    public GyroForward(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public boolean sonic (double inches, int desiredHeading) {
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftWheel.setPower(robot.PLATFORM_SPEED);
        robot.rightWheel.setPower(robot.PLATFORM_SPEED);
        int heading = robot.gyroSensor.getHeading();
        double offset = 0;
        while(robot.sonicOne.getDistance(DistanceUnit.INCH) > inches){
            heading = robot.gyroSensor.getHeading();
            if(desiredHeading == 0) {
                if (heading < 90) {
                    offset = robot.MOVEMENT_INCREMENT * heading;
                    robot.leftWheel.setPower(Range.clip(robot.PLATFORM_SPEED + offset, 0, 0.3));
                    robot.rightWheel.setPower(Range.clip(robot.PLATFORM_SPEED - offset, 0, 0.3));
                } else if (heading > 270) {
                    offset = robot.MOVEMENT_INCREMENT * (360 - heading);
                    robot.leftWheel.setPower(Range.clip(robot.PLATFORM_SPEED - offset, 0, 0.3));
                    robot.rightWheel.setPower(Range.clip(robot.PLATFORM_SPEED + offset, 0, 0.3));
                } else {
                    return false;
                }
            }
            else {
                offset = robot.MOVEMENT_INCREMENT * Math.abs((desiredHeading - heading));
                if (heading < desiredHeading) {
                    robot.leftWheel.setPower(Range.clip(robot.PLATFORM_SPEED - offset, 0, 0.3));
                    robot.rightWheel.setPower(Range.clip(robot.PLATFORM_SPEED + offset, 0, 0.3));
                } else if (heading > desiredHeading) {
                    robot.leftWheel.setPower(Range.clip(robot.PLATFORM_SPEED + offset, 0, 0.3));
                    robot.rightWheel.setPower(Range.clip(robot.PLATFORM_SPEED - offset, 0, 0.3));
                } else {
                    return false;
                }
            }
            telemetry.addData("offset", offset);
            telemetry.addData("heading", heading);
            telemetry.addData("left", robot.leftWheel.getPower());
            telemetry.addData("right", robot.rightWheel.getPower());
            telemetry.addData("sonic", robot.sonicOne.cmUltrasonic() / 2.54);
            telemetry.update();
        }
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return true;
    }

}
