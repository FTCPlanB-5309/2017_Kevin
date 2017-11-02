package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Gyro Testing", group="Testing")

public class TestingTheGyro extends LinearOpMode {

    //
    // Creating the robot class from the Plan B Hardware class and passing in telemetry
    // object as a parameter so that the hardware class can spit out telemetry data
    //
    RobotHardware         robot   = new RobotHardware(telemetry);
    Gyro                  gyro    = new Gyro(robot, telemetry);

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Wait until the gyro calibration is complete
        robot.gyroSensor.calibrate();
        timer.reset();
        while (!isStopRequested() && robot.gyroSensor.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
            telemetry.addData("Current Heading", robot.gyroSensor.getHeading());
            telemetry.update();
            sleep(3000);
            gyro.turn(45);
            sleep(3000);
            gyro.turn(315);
            sleep(3000);
            gyro.turn(0);

    }

}
