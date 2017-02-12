package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous-Red Balls And Beacons", group="Robot")
@SuppressWarnings("unused")
public class PlanRedBallAuto extends LinearOpMode {

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
        robot.rightShooter.setPower(0.675);
        robot.leftShooter.setPower(0.675);
        robot.sweeper1.setPower(-1);
        robot.backward(.8, 26);
        robot.kick();
        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.sweeper2.setPower(0);
        robot.sweeper1.setPower(0);
        robot.rightWheelTurn(1.0, 49);
        robot.backward(1.0, 62);
        robot.leftWheelTurn(-1, -41);
        Thread.sleep(250);
        robot.backward(1, 7);
        Thread.sleep(250);
        robot.leftWheelTurn(1.0, 85);
        Thread.sleep(250);
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
        robot.findLine(-.3);
        Thread.sleep(250);
        robot.forward(.3, 2);
        Thread.sleep(250);
        robot.CServo.setPosition(.6);
        Thread.sleep(1000);
        robot.poke(robot.RED, .4);
        Thread.sleep(250);
        //
        // Start looking for second Beacon
        //
        robot.forward(1.0, 36);
        robot.findLine(.3);
        robot.backward(.3, 2);
        robot.stopDrive(); // Just// in case
        robot.CServo.setPosition(.6);
        Thread.sleep(1000);
        robot.poke(robot.RED,.4);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        robot.stopDrive();
        sleep(1000);
        idle();
    }
}

