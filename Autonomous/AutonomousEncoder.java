package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;
import org.firstinspires.ftc.teamcode.NonOpModes.Extender;
import org.firstinspires.ftc.teamcode.NonOpModes.Point;
@Autonomous(name="EncoderTest", group="Linear Opmode")
//@Disabled
public class AutonomousEncoder extends LinearOpMode  {
    public RobotController r;
    @Override
    public void runOpMode() {
        // Support class for robot control
        r = new RobotController(this);
        r.aRamp = 1000.0;
        r.InitAutoWait();
        //r.DriveEncodeRamp(0,50,0.2, 0.4, 10);
        //r.DriveEncodeRamp(0,100,0.4, 0.25, 10);
        r.DriveEncode(0,10,0.2, 5);
        r.DriveEncode(-10,10,0.2, 5);
        r.DriveEncode(-20,10,0.2, 5);
        r.DriveEncode(-20,0,0.2, 5);
        r.setEndAngle(90);
        r.DriveEncodeRamp(50,0,0.16,0.4, 5);
        r.DriveEncodeRamp(100,0,0.4,0.16, 5);
        r.setEndAngle(0);
        r.DriveEncode(100,20,0.2,5);
        r.DriveEncode(100,0,0.2,5);
        r.setEndAngle(90);
        r.DriveEncode(80,0,0.2,5);
        r.setEndAngle(0);
        r.DriveEncode(50,0,0.2,5);
        r.DriveEncode(20,0,0.2,5);
        r.DriveEncode(0,0,0.2,5);
        


        // r.DriveEncode(30,50,0.25, 5);
        // r.setEndAngle(-20);
        // r.DriveEncode(30,0,0.25, 5);
        // r.DriveEncode(0,0,0.25, 5);
        // r.DriveEncode(0,50,0.25, 5);
        // r.setEndAngle(0);
        // r.DriveEncode(0,0,0.25, 5);
        // r.DriveEncode(30,0,0.25, 5);
        // r.DriveEncode(50,0,0.25, 5);
        // r.DriveEncode(0,0,0.25, 5);
        
    
        

    }
}
