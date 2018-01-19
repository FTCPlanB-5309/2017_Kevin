package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.media.MediaPlayer;
@Autonomous(name="Talking Robot", group="Sensors")





public class TalkingRobot extends LinearOpMode {

    //
    // Creating the robot class from the Plan B Hardware class and passing in telemetry
    // object as a parameter so that the hardware class can spit out telemetry data
    //
    RobotHardware         robot   = new RobotHardware(telemetry);
    private MediaPlayer mp = new MediaPlayer();
    private final String mediaPath = "/storage/emulated/0/Music";
    private final String mediaFile = "Jeoprady.mp3";

    public void runOpMode() throws InterruptedException {


            mp.start();
            try
            {
                mp = new MediaPlayer();
                mp.setDataSource(mediaPath + "/" + mediaFile);
                mp.prepare();
            }
            catch(Exception e)
            {
                telemetry.addData("Error" , "Oh no :(");
                telemetry.update();
            }
            sleep(1000);
            mp.stop();
            sleep(100);
            mp.start();
            sleep(38000);
    }
}