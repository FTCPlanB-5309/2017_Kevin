package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Forward {

    RobotHardware robot;
    Telemetry telemetry;

    public Forward(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void run(double speed, int distance) throws InterruptedException {
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Thread.sleep(250);
        int target = (int) (distance * robot.COUNTS_PER_INCH);
        robot.rightWheel.setTargetPosition(target);
        robot.leftWheel.setTargetPosition(target);
        robot.rightWheel.setPower(speed);
        robot.leftWheel.setPower(speed);


        while (robot.rightWheel.isBusy()) {
            Thread.yield();
        }

        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
    }
}

