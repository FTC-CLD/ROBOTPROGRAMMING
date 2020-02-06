package org.firstinspires.ftc.teamcode.Autonomous;

import java.io.File;
import java.util.List;

import java.io.FileNotFoundException;
import java.util.Scanner;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.io.FileWriter;
import java.io.IOException;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;
import org.firstinspires.ftc.teamcode.NonOpModes.Extender;
import org.firstinspires.ftc.teamcode.NonOpModes.Point;
@Autonomous(name="DemoAutonomous", group="Linear Opmode")
//@Disabled
public class AutonomousDemo extends LinearOpMode  {
    public RobotController r;
    private static String filename = "/sdcard/FIRST/java/src/Logging/Path.txt";



    public void readPath(String filename) throws FileNotFoundException 
    {
        //Get scanner instance
        Scanner scanner = new Scanner(new File(filename));
         
        //Set the delimiter used in file
        scanner.useDelimiter(",");
         
        //Get all tokens and store them in some data structure
        //I am just printing them
        while (scanner.hasNext()) 
        {
            int temp = Integer.valueOf(scanner.next());
            if(scanner.hasNext()) {
                telemetry.addData("xvalue", temp);
                r.DriveEncode( temp , Integer.valueOf(scanner.next()) ,0.2,5);
            }
            
        }
    }
    @Override
    public void runOpMode() {
        // Support class for robot control
        r = new RobotController(this);
        r.aRamp = 1000.0;
        r.InitAutoLog();
        r.InitAutoWait();
        try{
            readPath(filename);
        } catch (Exception e) {
            e.printStackTrace();
            telemetry.addData("","File not found");
        }
        telemetry.update();
        r.log1.close();

        
    
        

    }
}
