package org.firstinspires.ftc.teamcode.Unused;

import android.content.Context;
import com.qualcomm.robotcore.robot.Robot;
import java.io.ByteArrayOutputStream;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.io.InputStream;
import java.io.BufferedInputStream;



import com.qualcomm.robotcore.util.RobotLog; 

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;

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

@TeleOp(name="TestSocket", group="Concept")

public class TestSocket extends LinearOpMode {

    // List of available sound resources
    
    private Socket kkSocket = null;
    private PrintWriter outsocket = null;
   
            
    @Override
    public void runOpMode() {

        // Variables for choosing from the available sounds
        boolean prevA = false;
        double deltaT = 0;
        int counter = 0;
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        try {
            opensock();
        }
        catch (Exception e) {
                    RobotLog.vv("ERROR ",String.valueOf(e));
        }
        
        while (opModeIsActive()){
            
            if (gamepad1.b) telemetry.addData("Button B", "Pressed");
            // Display the current sound choice, and the playing status.
            boolean a = gamepad1.a;
            if (!prevA && a) {
                telemetry.addData("Button", "Pressed");
                RobotLog.vv("BUTTON ","Button A pressed");
                double t0 = runtime.time();
                try {
                    //testHTTP();
                     RobotLog.vv("LOGGING","before sendGET");
               
                     outsocket.println("data="+String.valueOf(gamepad1.left_stick_x));
                     
                    
                } catch(Exception e) {
                    RobotLog.vv("ERROR ",String.valueOf(e));
                    telemetry.addData("error", String.valueOf(e));
                      //  Block of code to handle errors
                }
                deltaT = runtime.time()-t0;
            }
            else telemetry.addData("Button", "Not Pressed");
            idle();
            telemetry.update();
            prevA = a;
            //prevgamepad1 = gamepad1;
        }
        
        closesock();
        
    }
    
    
    
  
    public   void opensock() throws IOException {
    
    String HOST = "192.168.49.175";
    try {
      kkSocket = new Socket(HOST, 4444);
      outsocket = new PrintWriter(kkSocket.getOutputStream(), true);
      outsocket.println("Sending request to Socket Server");
            
      RobotLog.vv("LOGGING","creating buffer reader...");
      //in = new BufferedReader(new InputStreamReader(kkSocket.getInputStream()));
      //RobotLog.vv("LOGGING","created ..." + in);
    } catch (UnknownHostException e) {
      RobotLog.vv("LOGGING ERR","Don't know about host: " + HOST + ".");
      // System.exit(1);
      return;
    } catch (IOException e) {
      RobotLog.vv("LOGGING ERR","Couldn't get I/O for the connection to: " + HOST + ".");
      // System.exit(1);
      return;
    }
    
    
    // int i=0;
    // while ((fromServer = in.readLine()) != null) {
    //     telemetry.addData("recved", "Server: " + fromServer);
    //     telemetry.update();
    //     sleep(1000);
    //   RobotLog.vv("LOGGING","Server: " + fromServer);
    //   }
    
  }
  
  private void closesock() {
      try {
          outsocket.close();
          
      kkSocket.close();
      } catch (Exception e) {
           RobotLog.vv("LOGGING ERR","close problem");
      }
  }
  
}

