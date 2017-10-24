package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TestingTheGyro {
    @Autonomous(name="Testing the Gyro", group="Andy")
    public class Auto_Template extends LinearOpMode {

        //
        // Creating the robot class from the Plan B Hardware class and passing in telemetry
        // object as a parameter so that the hardware class can spit out telemetry data
        //
        RobotHardware         robot   = new RobotHardware(telemetry);   // Use a Plan B Hardware Class
        Gyro                  gyro    = new Gyro(robot, telemetry);

        private ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() {

        /*
         * Initialize the robot hardware
         * The init() method of the hardware class does all the work here
         */
            robot.init(hardwareMap);
            robot.gyroSensor.calibrate();
            while (robot.gyroSensor.isCalibrating() == true) {sleep(10);}



            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");    //
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            gyro.turn(240);
            sleep(5000);

            //
            // This is where the individual steps of the auto program go
            // Most which start with robot....
            //

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    }
}
