package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;
import org.firstinspires.ftc.teamcode.NonOpModes.Extender;
@Autonomous(name="SkystoneFinder", group="Linear Opmode")

public class AutonomousSkystoneFinder extends LinearOpMode  {
    public RobotController r;
    public void GrabStone() {
        r.DriveDistance(0,7,0.24);
        r.DriveStop();
        r.manageExtender.enterNext();
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        r.DriveDistance(0,-22,0.24);

    }
    public int GrabSkystone(boolean isRed) {
        int alliance = isRed ? 1 : -1;
        if (r.IsSkystone()) {
            GrabStone();
            r.DriveRotate(-90*alliance);
            return 0;
        }
        r.DriveDistance(r.LStone*alliance,0, 0.24);
        if (r.IsSkystone()) {
            GrabStone();
            r.DriveRotate(-90*alliance);
            r.DriveDistance(0,r.LStone*alliance, 0.24);
            return 1;
        }
        r.DriveDistance(r.LStone*alliance, 0,0.24);
        GrabStone();
        r.DriveRotate(-90*alliance);
        r.DriveDistance(0,2*r.LStone*alliance, 0.24);
        return -3;
    }
    @Override
    public void runOpMode() {
        // Support class for robot control
        r = new RobotController(this);
        r.aRamp = 1000.0;
        r.InitAndWait();
        // Moves forward to the stones
        r.DriveDistance(0,67, 0.24);
        r.DriveStop();
        sleep(200);
        // Grabs a skystone
        int skystone = GrabSkystone(true);
        // Drives to the building area
        r.DriveRamp(0,75, 0.2,0.8);
        r.DriveDistance(0, 26, 0.8);
        // Lifts the extender
        r.manageExtender.enterNext();
        r.DriveDistance(0, 25, 0.8);
        r.DriveRamp(0,75, 0.8, 0.05);
        r.DriveRotate(0);
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        }
        r.DriveDistance(0,30,0.24);
        r.DriveStop();
        r.hooksDown();
        r.manageExtender.enterNext();
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        r.manageExtender.enterNext();
        r.DriveDistance(0, -54, 0.24);
        r.DriveRotate(-90);
        r.DriveStop();
        r.hooksUp();


        r.DriveDistance(0, -10, 0.24);
        r.DriveStop();
        r.manageExtender.enterNext();
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        
        r.DriveRamp(0, -70, 0.1, 0.8);
        r.DriveDistance(0, -80, 0.8);
        r.manageExtender.enterNext();
        r.DriveRamp(0, -80-skystone*r.LStone, 0.8, 0.2);
        r.DriveRotate(0);
        // Grabs a second stone
        r.DriveDistance(0, 20, 0.24);
        GrabStone();
        r.DriveRotate(-90);
        // Drives the second time to the foundation
        r.DriveRamp(0,75, 0.2,0.8);
        r.DriveDistance(0, 70+skystone*r.LStone, 0.8);
        // Lifts the extender
        r.manageExtender.enterNext();
        r.DriveDistance(0, 15, 0.8);
        r.DriveRamp(0,75, 0.8, 0.1);
        // placestone on the foundation
        r.manageExtender.enterNext();
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        r.manageExtender.enterNext();
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        // Arm omlaag
        r.DriveRamp(0,-45, 0.2,0.8);
        r.manageExtender.enterNext();
        r.DriveRamp(0,-50, 0.8,0.2);
        

    }
}