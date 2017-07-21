package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Purple Autonomous", group="Robot")
@SuppressWarnings("unused")
public class PlanBlueBackup extends LinearOpMode {

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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        Thread.sleep(20000);
        robot.rightShooter.setPower(1);
        robot.leftShooter.setPower(1);
        robot.sweeper1.setPower(-.5);
        robot.backward(2500, 44, 4000);
        robot.kick();
        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.sweeper2.setPower(0);
        robot.sweeper1.setPower(0);
        robot.backward(3000, 24, 4000);
        robot.stopDrive();
        idle();
    }
}
