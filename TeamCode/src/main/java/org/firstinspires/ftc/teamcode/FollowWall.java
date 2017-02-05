package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="FollowWall", group="Robot")
@Disabled
@SuppressWarnings("unused")
public class FollowWall extends LinearOpMode {
    static final int TURN_SPEED = 200;
    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waitirng;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double distance = robot.rangeRead();
        double distance2 = robot.rangeRead2();
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        for (int i = 1; i < 10 && distance2 == 255; i++) {
            if (distance == 255)
                distance2 = robot.rangeRead2();
        }
        if (distance2 == 255) {
            distance2 = 10;
        }
        robot.leftWheel.setMaxSpeed(2000);
        robot.rightWheel.setMaxSpeed(2000);
        robot.rightWheel.setPower(0.5);
        robot.leftWheel.setPower(0.5);
        int leftWheelSpeed;
        int rightWheelSpeed;
        double difference;
        difference = Math.abs(12 - distance2);
        while (robot.leftWheel.getCurrentPosition() < 30000){
            if(distance2 == 255){
                leftWheelSpeed = 700;
                rightWheelSpeed = 700;
            }
            else if(distance2 > 13){
                rightWheelSpeed = TURN_SPEED - (int)(difference * 100);
                leftWheelSpeed = TURN_SPEED + (int)(difference * 100);
            }
            else if(distance2 < 12){
                rightWheelSpeed = TURN_SPEED + (int)(difference * 100);
                leftWheelSpeed = TURN_SPEED - (int)(difference * 100);
            }
            else{
                leftWheelSpeed = 3000;
                rightWheelSpeed = 3000;
            }
            robot.rightWheel.setMaxSpeed(rightWheelSpeed);
            robot.leftWheel.setMaxSpeed(leftWheelSpeed);

            distance2 = robot.rangeRead2();
            if(distance2 != 255)
                difference = Math.abs(12 - distance2);
            telemetry.addData("Distance 2", distance2);
            telemetry.addData("Difference", difference);
            telemetry.addData("Left Speed", robot.leftWheel.getMaxSpeed());
            telemetry.addData("Right Speed", robot.rightWheel.getMaxSpeed());
            telemetry.update();
        }
        robot.stopDrive();
    }
}