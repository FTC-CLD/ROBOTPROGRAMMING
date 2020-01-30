package org.firstinspires.ftc.teamcode.Unused;

import android.content.Context;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;

import java.io.BufferedWriter;
import java.io.File;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.io.BufferedReader;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.InputStream;
import java.io.BufferedInputStream;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.FileWriter;

import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//import org.firstinspires.ftc.teamcode.NonOpModes.HTTPLogger;
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
@Disabled
@TeleOp(name="HTTPtestert", group="Concept")

public class HTTPtest extends LinearOpMode {
      String url="http://192.168.49.175:8888";
      //private HTTPLogger httplog;
    
      private ElapsedTime runtime; 
        
      private File file=null;
      private BufferedWriter output=null;
      //private FileOutputStream fOut=null;
      
      //private OutputStreamWriter osw=null;
      //Context context=null;
            
            
    @Override
    public void runOpMode() {
      
        //httplog = new HTTPLogger(url, this);
        String text = "Hello world";
        //context = getApplicationContext();
        //context = this;
        ElapsedTime runtime = new ElapsedTime();
        
        waitForStart();
        
        try {
            
            //FileOutputStream fOut = context.openFileOutput("/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/example.csv",
            //                                                    "MODE_WORLD_READABLE");
            //OutputStreamWriter osw = new OutputStreamWriter(fOut); 
            
            File file = new File("/sdcard/FIRST/java/src/Logging/example2.txt"); //,"MODE_WORLD_READABLE");
            output = new BufferedWriter(new FileWriter(file));
            output.write("t,joyx,joyy\n");
            //osw.write(text);
        } catch ( IOException e ) {
            e.printStackTrace();
        }
        
        boolean prevA = false;
        while (opModeIsActive()){
            boolean a = gamepad1.a;
            //counter++;
            if (a && !prevA) { 
                double t0 = runtime.time();
                telemetry.addData("Button A", "Pressed");
                telemetry.addData("left stick", String.valueOf(gamepad1.left_stick_x));
                
                telemetry.update();
                //telemetry.addData("error", httplog.clear());
                //httplog.logvar("x",gamepad1.left_stick_x);
                try {
                //osw.write("hallo button A");
                output.write(String.valueOf(t0)+","+String.valueOf(gamepad1.left_stick_x)+","+String.valueOf(gamepad1.left_stick_y)+"\n");
                }catch ( IOException e ) {
            e.printStackTrace();
        }
                
                sleep(500);

            }
            telemetry.addData("Button A", "NotPressed");
            prevA = a;
            idle();
            telemetry.update();
        }
        
    
    
    try {
         output.close();
    //osw.close();
    
    }
    catch ( IOException e ) {
            e.printStackTrace();
        }
    }
}

