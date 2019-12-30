package org.firstinspires.ftc.teamcode.TeleOperated;

import android.content.Context;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.io.InputStream;
import java.io.BufferedInputStream;

import com.qualcomm.ftccommon.SoundPlayer;
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
    
    BNO055IMU imu;
        
            
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Variables for choosing from the available sounds
       Gamepad prevgamepad1 = gamepad1;
        boolean prevA = false;
        double deltaT = 0;
        int counter = 0;
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        
        
        while (opModeIsActive()){
            
            //counter++;
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            
            double heading = (angles.firstAngle);
            
            counter++;
            if (gamepad1.b) telemetry.addData("Button B", "Pressed");
            // Display the current sound choice, and the playing status.
            boolean a = gamepad1.a;
            // if (!prevA && a) {
                telemetry.addData("Button", "Pressed");
                double t0 = runtime.time();
                try {
                    if (sendGET("y="+String.valueOf(heading)+"&x="+String.valueOf(t0))==1) {
                        telemetry.addData("HTTP", "Succes");
                    } 
                    else {
                        telemetry.addData("HTTP", "Fail");
                    }
                    
                } catch(Exception e) {
                      //  Block of code to handle errors
                }
                deltaT = runtime.time()-t0;
            // }
            //else telemetry.addData("Button", "Not Pressed");
            sleep(100);
            telemetry.update();
            prevA = a;
            //prevgamepad1 = gamepad1;
        }
        
    }
    
    
    private static int sendGET(String param) throws IOException {
        URL obj = new URL("http://192.168.49.175:8081?"+param);
        HttpURLConnection con = (HttpURLConnection) obj.openConnection();
        con.setRequestMethod("GET");
        //con.setRequestProperty("User-Agent", USER_AGENT);
        int responseCode = con.getResponseCode();
        System.out.println("GET Response Code :: " + responseCode);
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

