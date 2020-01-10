package org.firstinspires.ftc.teamcode.NonOpModes;

import java.io.InputStream;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/**
 * commands send to the server:
 * ?cmd=clear
 * ?cmd=copyfile&filename=<new file>
 * ?cmd=logvar&t=<timestamp>&<varname>=<value>
 */


public class HTTPLogger {
    private String remoteurl;
    private LinearOpMode opMode;
    private ElapsedTime runtime;


    /**
     * Initialize an HTTP logger
     * @param url the url of the HTTP web server
     */
    public  HTTPLogger(String url, LinearOpMode opM) {
        remoteurl= url;
        opMode = opM;

    }

    /**
     * Clears the logged data
     */
    public String clear() { // clear 
        try {
            if (sendGET("cmd=clear")) {
                return "Success";
            }
            else {
                return "Failure";
            }
        }
        catch(Exception e) {
            return String.valueOf(e);
            //  Block of code to handle errors
        }
    }

    /**
     * Send a single variable value to http server
     * @param varname name of the variable, .e.g. "x", or "heading"
     * @param value value of the measurement as a double
     * @return
     */

    public boolean logvar(String varname, float value) {
       double t0 = runtime.time();
       boolean success=false;
        try {
            success=sendGET("cmd=logvar&t="+String.valueOf(t0)+"&"+varname+"="+String.valueOf(value));
        } catch(Exception e) {
              //  Block of code to handle errors
        }
        return success;
    }
    
    /**
     * Make a copy of the log file on the server
     * @param newfilename
     */

    public void copylogfile(String newfilename){
        try {
            sendGET("cmd=copyfile&filename="+newfilename);
        }
        catch(Exception e) {
            //  Block of code to handle errors
        }
    }
    
    private boolean sendGET(String param) throws IOException {
        URL obj = new URL(remoteurl+"/?"+param);
        opMode.telemetry.addData("Starting http Get",String.valueOf(obj));
        opMode.telemetry.update();
        HttpURLConnection con = (HttpURLConnection) obj.openConnection();
        con.setRequestMethod("GET");
        //con.setRequestProperty("User-Agent", USER_AGENT);
        int responseCode = con.getResponseCode();
         //opMode.telemetry.addData("GET Response Code :: " + responseCode);
        if (responseCode == HttpURLConnection.HTTP_OK) { // success
            // BufferedReader in = new BufferedReader(new InputStreamReader(
            //         con.getInputStream()));
            // String inputLine;
            // StringBuffer response = new StringBuffer();

            // while ((inputLine = in.readLine()) != null) {
            //     response.append(inputLine);
            // }
            // in.close();

            // // print result
             return(true);
        } else {
            return(false);
        }

    }
   
}
