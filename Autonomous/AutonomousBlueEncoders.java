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
@Autonomous(name="BlueEncoders", group="Linear Opmode")
//@Disabled
public class AutonomousBlueEncoders extends LinearOpMode  {
    public RobotController r;

    
    public void GrabStone(int stone, int alliance) {
        r.DriveEncode( alliance * stone*r.LStone, 80 ,0.3,1);
        r.DriveStop();
        r.manageExtender.enterNext();
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        r.DriveEncodeRamp( alliance * stone*r.LStone, 67,0.24,0.32,1);

    }
    public int GrabSkystone(boolean isRed) {
        int alliance = isRed ? 1 : -1;
        int stone = 0;
        
        r.DriveEncodeRamp( 0 , 25 ,0.2,0.9, 0);
        r.DriveEncodeRamp( 0 , 64 ,0.9,0.16, 0);
        r.DriveEncodeRamp( 0 , 68 ,0.9,0.15, 0);
        r.DriveStop();
        if (r.IsSkystone()) {
            GrabStone(stone, alliance);
            return 3;
        }
        stone++;
        r.DriveEncode(stone*alliance*r.LStone,68, 0.27,0);
        if (r.IsSkystone()) {
            r.DriveStop();
            GrabStone(stone,alliance);
            return 4;
        }
        stone++;
        r.DriveEncode(stone*alliance*r.LStone,68, 0.27,2);
        GrabStone(stone, alliance);
        return 0;
    }
        @Override
    public void runOpMode() {
        // Support class for robot control
        r = new RobotController(this);
        r.aRamp = 1000.0;
        r.InitAutoWait();
        r.hooksUp();
        int skystone = GrabSkystone(false);
        // Skystone herkennen
        r.setEndAngle(-90);
        r.DriveEncodeRamp( 40 , 58 ,0.3,1,5);
        
        r.DriveEncode( 92 , 58 ,1,1);
        r.manageExtender.enterNext();
        //lift omhoog
        r.setEndAngle(0);
        r.DriveEncodeRamp( 189 , 67 ,1,0.2,7);
        r.DriveEncode( 190 , 93 ,0.4,0);
        r.DriveStop();
        // pak plaat en stone erop
        r.hooksDown();
        r.manageExtender.enterNext();
        r.manageExtender.enterNext();
        r.DriveRotate(-10);
        
        r.DriveEncode( 177 , 49 ,0.5,1);
        r.DriveRotate(-90);
        // lift ergens inklappen
        r.hooksUp();
        sleep(100);
        r.DriveEncodeRamp( 155 , 58 ,0.3,0.7,1);
        r.manageExtender.enterNext();
        r.DriveEncodeRamp( 126 , 58 ,0.7,1,1);
        r.DriveEncode( 80 , 58 ,1,1);
        r.setEndAngle(0);
        r.manageExtender.enterNext();
        r.DriveEncode( 50 , 58 ,1,1);
        r.DriveEncodeRamp(-skystone * r.LStone , 57 ,1,0.3,7);
        GrabStone(skystone, -1);
        r.setEndAngle(-90);
        r.DriveEncodeRamp( 40 , 58 ,0.3,1,5);
        
        r.DriveEncode( 92 , 58 ,1,1);
        r.manageExtender.enterNext();
        r.DriveEncodeRamp( 171 , 58,1,0.3,1);
        //Place second stone
        r.manageExtender.enterNext();
        r.manageExtender.enterNext();

        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        
        r.DriveEncodeRamp( 150 , 58,0.3,1,1);
        r.manageExtender.enterNext();
        r.DriveEncodeRamp( 92 , 58 ,1,0.3,1);
        r.DriveStop();

        

        
    
        

    }
}
