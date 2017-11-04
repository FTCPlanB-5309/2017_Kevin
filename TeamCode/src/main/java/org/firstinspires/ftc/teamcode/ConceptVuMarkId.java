package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by mgold on 10/31/2017.
 */

public class ConceptVuMarkId {

    Telemetry telemetry;
    VuforiaLocalizer vuforia;
    RelicRecoveryVuMark columnPosition = RelicRecoveryVuMark.UNKNOWN;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    public ConceptVuMarkId(HardwareMap hwMap, Telemetry telemetry)
    {
        int cameraMonitorViewId;
        this.telemetry = telemetry;



        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        /*
         * Set the license key
         */
        parameters.vuforiaLicenseKey = "AYboCe//////AAAAGZ74ine8W01csxwSLrlrFU8SCehSLWIwpwUNici0XIz9RK5ZR9rkALDwHUE" +
                "/860ACZBO2RMrFbWIFC27Ri6dW0fsZH24ckZfYxsXdmBYmch/6XuhsWx75wLnb7+4ZthJQLMZ0wfORFa+6bCn6xDvR" +
                "GIM+0wqrKZOEv1J+F16mj3MlG4Mx65/DBFc2t8ag9LoVoE2gCcKX+HmeA1aXfAWJVFfp1nhXXUMMyftFe1zaexsXoVbvuCc" +
                "pyz8aqqZpXBBBuBoRGsxfzuQ6Tq2UlvzzFazzDdFliwmYFdmAQyfae7HORmTHTZs31eVlOB+cJPzciASiUP+KQz7lzRidcS" +
                "ruR3U06PYPv9P1uM9vf4KYwdY";

        /*
         * Setting the camera to the front camera which is facing away from the robot
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    public RelicRecoveryVuMark findColumn (int MaxTimeInMilliseconds)throws InterruptedException
    {

        ElapsedTime runTime = new ElapsedTime();


        relicTrackables.activate();  // Start looking for VuMark
        columnPosition = RelicRecoveryVuMark.from(relicTemplate);
        runTime.reset();

        while ( columnPosition == RelicRecoveryVuMark.UNKNOWN &&
                runTime.milliseconds() < MaxTimeInMilliseconds) {
            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            columnPosition = RelicRecoveryVuMark.from(relicTemplate);
            if (columnPosition != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", columnPosition);
                telemetry.update();
            }
            else {
                telemetry.addData("VuMark", "not visible");
                telemetry.update();
            }
            telemetry.addData("Time Looking", runTime.toString());
            telemetry.update();
        }
        return columnPosition;
    }
}
