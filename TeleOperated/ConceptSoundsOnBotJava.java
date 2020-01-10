package org.firstinspires.ftc.teamcode.TeleOperated;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.File;
import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;

/**
 * This file demonstrates how to play simple sounds on both the RC and DS phones.
 * It illustrates how to play sound files that have been copied to the RC Phone
 * This technique is best suited for use with OnBotJava since it does not require the app to be modified.
 *
 * Operation:
 *
 * Gamepad X & B buttons are used to trigger sounds in this example, but any event can be used.
 * Note: Time should be allowed for sounds to complete before playing other sounds.
 *
 *  To play a new sound, you will need to copy the .wav files to the phone, and then provide the full path to them as part of your OpMode.
 *  This is done in this sample for the two sound files.  silver.wav and gold.wav
 *
 *  You can put the files in a variety of soundPaths, but we recommend you put them in the /FIRST/blocks/sounds folder.
 *  Your OpModes will have guaranteed access to this folder, and you can transfer files into this folder using the BLOCKS web page.
 *  --  There is a link called "sounds" on the right hand side of the color bar on the BLOCKS page that can be used to send sound files to this folder by default.
 *  Or you can use Windows File Manager, or ADB to transfer the sound files
 *
 *  To get full use of THIS sample, you will need to copy two sound file called silver.wav and gold.wav to /FIRST/blocks/sounds on the RC phone.
 *  They can be located here:
 *      https://github.com/ftctechnh/ftc_app/tree/master/FtcRobotController/src/main/res/raw/gold.wav
 *      https://github.com/ftctechnh/ftc_app/tree/master/FtcRobotController/src/main/res/raw/silver.wav
 */

@TeleOp(name="Concept: Sound Files", group="Concept")

public class ConceptSoundsOnBotJava extends LinearOpMode {

    // Point to sound files on the phone's drive
    private String soundPath = "/FIRST/blocks/sounds";
    private File goldFile   = new File("/sdcard" + soundPath + "/Skystone.wav");
    private File silverFile = new File("/sdcard" + soundPath + "/Normalstone.wav");

    // Declare OpMode members.
    private boolean isX = false;    // Gamepad button state variables
    private boolean isB = false;

    private boolean wasX = false;   // Gamepad button history variables
    private boolean WasB = false;
    public RobotController r;

    


    @Override
    public void runOpMode() {

        // Support class for robot control
        r = new RobotController(this);
        r.Init();
        
        // Make sure that the sound files exist on the phone
        boolean goldFound   = goldFile.exists();
        boolean silverFound = silverFile.exists();

        // Display sound status
        telemetry.addData("gold sound",   goldFound ?   "Found" : "NOT Found \nCopy gold.wav to " + soundPath  );
        telemetry.addData("silver sound", silverFound ? "Found" : "NOT Found \nCopy silver.wav to " + soundPath );

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Start to continue");
        telemetry.update();
        waitForStart();

        telemetry.addData(">", "Press X or B to play sounds.");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // say Gold each time gamepad B is pressed  (This sound is a resource)
            if (goldFound && (isB = gamepad1.b) && !WasB) {
                if (r.IsSkystone()) {
                   SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldFile); 
                }
                else {
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverFile);
                }
                
                telemetry.addData("Playing", "Gold File");
                telemetry.update();
            }

            // Save last button states
            wasX = isX;
            WasB = isB;
        }
    }
} 