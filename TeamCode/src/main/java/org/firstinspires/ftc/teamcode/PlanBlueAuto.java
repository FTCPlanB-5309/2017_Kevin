package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous-Blue Beacons Only", group="Robot")
@Disabled
@SuppressWarnings("unused")
public class PlanBlueAuto extends LinearOpMode {

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
        robot.backward(1, 86);
        robot.rightTankTurn(-1, -65);
        Thread.sleep(250);
        robot.backward(1, 7);
        robot.leftWheelTurn(35, 79);
        Thread.sleep(250);
        robot.findLine(-.4);
        Thread.sleep(250);
        robot.poke(robot.BLUE, .3);
        robot.forward(1, 36);
        robot.findLine(.4);
        robot.poke(robot.BLUE, .3);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        robot.stopDrive();
        sleep(1000);
        idle();
    }
}