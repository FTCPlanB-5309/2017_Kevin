package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Left Blue Autonomous", group="Blue")

public class BlueLeftAuto extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot   = new RobotHardware();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        Thread.sleep(5000);
        robot.Jewel(robot.BLUE);

    }

}
