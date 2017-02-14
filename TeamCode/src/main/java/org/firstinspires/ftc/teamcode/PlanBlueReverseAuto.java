package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Blue Auto", group = "Robot")
@SuppressWarnings("unused")
public class PlanBlueReverseAuto extends LinearOpMode {

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

        robot.shooterSpeed();
        robot.sweeper1.setPower(-1);
        robot.backward(1500, 26, 3000);
        robot.kick();
        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.sweeper2.setPower(0);
        robot.sweeper1.setPower(0);
        //
        // Drive to Beacon wall
        //
        robot.leftWheelTurn(1500, 49, 2000);
        Thread.sleep(250);
        robot.backward(1750, 58, 6000);
        robot.rightWheelTurn(2500, -51, 2000);
        Thread.sleep(250);
        robot.backward(1500, 7, 1000);
        robot.leftWheelTurn(1500, 85, 2500);
        Thread.sleep(250);
        double cmDistance = robot.rangeRead();
        double cmDistance2 = robot.rangeRead2();
        telemetry.addData("CM Distance pre 1", cmDistance);
        telemetry.addData("CM Distance pre 2", cmDistance2);
        telemetry.update();
        robot.adjustTurn(600);
        Thread.sleep(250);
        cmDistance = robot.rangeRead();
        cmDistance2 = robot.rangeRead2();
        telemetry.addData("CM Distance post 1", cmDistance);
        telemetry.addData("CM Distance post 2", cmDistance2);
        telemetry.update();
        //
        // Find first Beacon
        //
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftWheel.setPower(.3);
        robot.rightWheel.setPower(.3);
        while (robot.OpticalSensor.getLightDetected() < .8 && opModeIsActive()) {
            Thread.yield();
        }
        // Stop all motion;
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backward(1000, 2, 750);
        robot.CServo.setPosition(.6);
        Thread.sleep(750);
        robot.poke(robot.BLUE, 1200);
        robot.forward(1500, 36, 3000);
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftWheel.setPower(.3);
        robot.rightWheel.setPower(.3);
        while (robot.OpticalSensor.getLightDetected() < .3 && opModeIsActive()) {
            Thread.yield();
        }
        // Stop all motion;
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backward(1000, 1, 500);
        robot.CServo.setPosition(.6);
        Thread.sleep(750);
        robot.poke(robot.BLUE, 1200);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        robot.stopDrive();
        Thread.sleep(3000);
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