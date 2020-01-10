package org.firstinspires.ftc.teamcode.TeleOperated;

import org.firstinspires.ftc.teamcode.NonOpModes.LogFile;
import java.io.File;

import com.qualcomm.robotcore.util.RobotLog; 

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file demonstrates how to play one of the several SKYSTONE/Star Wars sounds loaded into the SDK.
 * It does this by creating a simple "chooser" controlled by the gamepad Up Down buttons.
 * This code also prevents sounds from stacking up by setting a "playing" flag, which is cleared when the sound finishes playing.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * Operation:
 *      Use the DPAD to change the selected sound, and the Right Bumper to play it.
 */

@TeleOp(name="TestLogFile", group="Concept")

public class TestLogFile extends LinearOpMode {

    // List of available sound resources
    
  
            
    @Override
    public void runOpMode() {

        // Variables for choosing from the available sounds
        boolean prevA = false;
        double deltaT = 0;
        int counter = 0;
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        LogFile log1 = new LogFile();
        log1.init("logging1.txt");
        log1.write("time,x,y");
        while (opModeIsActive()){
            
            if (gamepad1.b) telemetry.addData("Button B", "Pressed");
            // Display the current sound choice, and the playing status.
            boolean a = gamepad1.a;
            if (!prevA && a) {
                telemetry.addData("Button", "Pressed");
                RobotLog.vv("BUTTON ","Button A pressed");
                double t0 = runtime.time();
                RobotLog.vv("LOGGING","before sendGET");
                log1.write(String.valueOf(t0)+","+String.valueOf(gamepad1.left_stick_x)+","+String.valueOf(gamepad1.left_stick_y)+"\n");
                log1.flush();
                     
                    
                deltaT = runtime.time()-t0;
            }
            else telemetry.addData("Button", "Not Pressed");
            idle();
            telemetry.update();
            prevA = a;
            //prevgamepad1 = gamepad1;
        }
        
        log1.close();
        
    }
    
  
}

