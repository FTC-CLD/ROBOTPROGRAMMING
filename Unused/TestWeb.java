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

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

@TeleOp(name="HTTP test", group="Concept")

public class TestWeb extends LinearOpMode {

    // List of available sound resources
    
        
            
    @Override
    public void runOpMode() {

        // Variables for choosing from the available sounds
        boolean prevA = false;
        double deltaT = 0;
        int counter = 0;
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        
        
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
               
                     if (sendGET("y="+String.valueOf(gamepad1.left_stick_x)+"&x="+String.valueOf(t0))==1) {
                         telemetry.addData("HTTP", "Succes");
                     } 
                     else {
                         telemetry.addData("HTTP", "Fail");
                     }
                    
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
        
    }
    
    private String readStream(InputStream is) {
    try {
      ByteArrayOutputStream bo = new ByteArrayOutputStream();
      int i = is.read();
      while(i != -1) {
        bo.write(i);
        i = is.read();
      }
      return bo.toString();
    } catch (IOException e) {
      return "";
    }
}
    
    private void testHTTP()  throws Exception  {
         URL url = new URL("http://192.168.49.175:8888/jnk");
   HttpURLConnection urlConnection = (HttpURLConnection) url.openConnection();
   try {
     InputStream in = new BufferedInputStream(urlConnection.getInputStream());
     readStream(in);
   } finally {
     urlConnection.disconnect();
   }
    }
    
    
    private int sendGET(String param) throws Exception {
        URL obj = new URL("https://192.168.49.175:8888/hallo");
        HttpURLConnection con = null;
        con = (HttpURLConnection) obj.openConnection();
        telemetry.addData("Connection","") ;
        telemetry.update();
        con.setRequestMethod("GET");
        telemetry.addData("method get","") ;
        telemetry.update();
        RobotLog.vv("LOGGING ","before con.setrequest");
        sleep(100);
        con.setRequestProperty("User-Agent", "Android");
        RobotLog.vv("LOGGING ","after con.setrequest");
       
            int responseCode = con.getResponseCode();
             RobotLog.vv("LOGGING ","after con.getResponeCode");
       
            telemetry.addData("responseCode","") ;
            telemetry.update();
            
            if (responseCode == HttpURLConnection.HTTP_OK) { // success
                BufferedReader in = new BufferedReader(new InputStreamReader(
                        con.getInputStream()));
                String inputLine;
                StringBuffer response = new StringBuffer();
    
                while ((inputLine = in.readLine()) != null) {
                    response.append(inputLine);
                }
                in.close();
    
                // print result
                 return(1);
            } else {
                return(0);
            }

    }
}

