package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.media.MediaPlayer;


/**
 * Created by mgold on 1/22/2018.
 */

@Autonomous(name="CarterTalks", group="Other")

public class CarterTalks extends LinearOpMode {
    //
    // Make Carter talk
    //
    // The mynameiscarter wav file has to be copied to new folder under res called raw for this
    // to work.

    private MediaPlayer myMediaPlayer;

    public void runOpMode() throws InterruptedException {
        MediaPlayer mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.sirhowardcarter);
        mediaPlayer.start();
    }
}
