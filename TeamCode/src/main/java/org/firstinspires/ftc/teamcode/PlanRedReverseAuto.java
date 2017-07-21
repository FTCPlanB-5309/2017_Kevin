package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Auto", group="Robot")
@SuppressWarnings("unused")
public class PlanRedReverseAuto extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //int thisColor;
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
        robot.rightWheelTurn(1500, 49, 3000);
        Thread.sleep(250);
        robot.backward(1750, 58, 6000);
        robot.leftWheelTurn(1500, -37, 2500);
        Thread.sleep(250);
        robot.backward(3000, 7, 1000);
        robot.leftWheelTurn(1500, 85, 3000);
        Thread.sleep(250);
        double cmDistance = robot.rangeRead();
        double cmDistance2 = robot.rangeRead2();
        telemetry.addData("CM Distance pre 1", cmDistance);
        telemetry.addData("CM Distance pre 2", cmDistance2);
        telemetry.update();
        int returnAdj = robot.matrixAdjustment(600);
        Thread.sleep(250);
        telemetry.addData("Adjustment Value", returnAdj);
        telemetry.update();
        returnAdj = robot.matrixAdjustment(600);
        Thread.sleep(250);
        telemetry.update();
        cmDistance = robot.rangeRead();
        cmDistance2 = robot.rangeRead2();
        telemetry.addData("CM Distance post 1", cmDistance);
        telemetry.addData("CM Distance post 2", cmDistance2);
        telemetry.update();
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
        robot.backward(1000, 1, 750);
        robot.CServo.setPosition(.6);
        Thread.sleep(750);
        robot.poke(robot.RED, 1200);
        robot.leftWheelTurn(600, 1, 750);
        //
        // Start looking for second Beacon
        //
        robot.backward(1500, 36, 3000);
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftWheel.setPower(-.3);
        robot.rightWheel.setPower(-.3);
        while (robot.OpticalSensor.getLightDetected() < .8 && opModeIsActive()) {
            Thread.yield();
        }
        // Stop all motion;
        robot.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.forward(1000, 3, 500);
        robot.stopDrive(); // Just// in case
        robot.CServo.setPosition(.6);
        Thread.sleep(750);
        robot.poke(robot.RED,1200);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        robot.stopDrive();
        sleep(3000);
        idle();
    }
}

