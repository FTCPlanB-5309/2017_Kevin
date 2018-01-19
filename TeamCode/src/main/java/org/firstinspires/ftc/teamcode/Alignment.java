package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



/**
 * Created by user on 11/1/2017.
 */
@Autonomous(name="Alignment", group="Sensor")

public class Alignment extends LinearOpMode {

    RobotHardware         robot   = new RobotHardware(telemetry);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Distance", (robot.sonicTwo.cmUltrasonic()));
            telemetry.update();
        }
    }
}


