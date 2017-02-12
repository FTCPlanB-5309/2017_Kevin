package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous-Blue Balls And Beacons", group="Robot")
@SuppressWarnings("unused")
public class PlanBlueBallAuto extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();

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

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        telemetry.addData("1", "Left Drive Power", robot.leftWheel.getPower());
        telemetry.addData("2", "Right Drive Power", robot.rightWheel.getPower());

        robot.rightShooter.setPower(0.675);
        robot.leftShooter.setPower(0.675);
        robot.sweeper1.setPower(-1);
        robot.backward(.8, 26);
        robot.kick();
        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.sweeper2.setPower(0);
        robot.sweeper1.setPower(0);
        //
        // Drive to Beacon wall
        //
        robot.leftWheelTurn(1, 49);
        robot.backward(1, 64);
        robot.rightWheelTurn(-1, -51);
        Thread.sleep(250);
        robot.backward(1, 7);
        robot.leftWheelTurn(1, 86);
        double cmDistance = robot.rangeRead();
        double cmDistance2 = robot.rangeRead2();
        telemetry.addData("CM Distance pre 1", cmDistance);
        telemetry.addData("CM Distance pre 2", cmDistance2);
        telemetry.update();
        robot.adjustTurn(500);
        cmDistance = robot.rangeRead();
        cmDistance2 = robot.rangeRead2();
        telemetry.addData("CM Distance post 1", cmDistance);
        telemetry.addData("CM Distance post 2", cmDistance2);
        telemetry.update();
        //
        // Find first Beacon
        //
        robot.findLine(.4);
        robot.backward(.4, 2);
        Thread.sleep(250);
        robot.CServo.setPosition(.6);
        Thread.sleep(1000);
        robot.poke(robot.BLUE, .3);
        robot.backward(1.0, 42);
        robot.findLine(-.4);
        robot.forward(.4, 2);
        robot.CServo.setPosition(.6);
        Thread.sleep(1000);
        robot.poke(robot.BLUE, .3);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        robot.stopDrive();
        idle();
    }
/*
    public void adjustTurn(int speed) throws InterruptedException {
        double distance = 255;
        double distance2 = 255;
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        for (int i = 1; i < 10 && (distance == 255 || distance2 == 255); i++) {
            if (distance == 255)
                distance = robot.rangeRead();
            if (distance2 == 255)
                distance2 = robot.rangeRead2();
            Thread.yield();
        }
        int difference = (int) distance2 - (int) distance;
        if (distance == 255 || distance2 == 255) {
            difference = 0;
        }
        int setSpeed;
        if (difference == 0)
            setSpeed = 0;
        else
            setSpeed = speed + ((Math.abs(difference)-1) * 200);
        while (difference!=0){
            if (distance > distance2) {
                robot.leftWheel.setMaxSpeed(setSpeed);
                robot.rightWheel.setMaxSpeed(setSpeed);
                robot.leftWheel.setPower(-0.5);
                robot.rightWheel.setPower(0.5);
            }
            else {
                robot.leftWheel.setMaxSpeed(setSpeed);
                robot.rightWheel.setMaxSpeed(setSpeed);
                robot.leftWheel.setPower(0.5);
                robot.rightWheel.setPower(-0.5);
            }
            distance = robot.rangeRead();
            distance2 = robot.rangeRead2();
            difference = (int) distance2 - (int) distance;
            setSpeed = speed + ((Math.abs(difference)-1) * 200);
            telemetry.addData("Distance", distance);
            telemetry.addData("Distance 2", distance2);
            telemetry.addData("Left Power", robot.leftWheel.getPower());
            telemetry.addData("Right Power", robot.rightWheel.getPower());
            telemetry.addData("Left Speed", robot.leftWheel.getMaxSpeed());
            telemetry.addData("Right Speed", robot.rightWheel.getMaxSpeed());
            telemetry.update();
        }
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }*/
}