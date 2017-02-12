package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous-Red Beacons Only", group="Robot")
@Disabled
@SuppressWarnings("unused")
public class PlanRedAuto extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        int thisColor;
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
        robot.backward(1, 86);
        robot.leftTankTurn(-1, -65);
        Thread.sleep(250);
        robot.backward(1, 7);
        robot.leftWheelTurn(1, 79);
        Thread.sleep(250);
        robot.findLine(.4);
        Thread.sleep(250);
        robot.poke(robot.RED,.4);
        robot.backward(1, 36);
        robot.findLine(-.4);
        robot.poke(robot.RED,.4);
        sleep(1000);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        robot.stopDrive();
        sleep(1000);
        idle();
    }
}

