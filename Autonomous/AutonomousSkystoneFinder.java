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
        r.DriveDistance(0,5,0.24);
        r.DriveStop();
        r.manageExtender.enterNext();
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        r.DriveDistance(0,-15,0.24);

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
        return 1;
    }
    @Override
    public void runOpMode() {
        // Support class for robot control
        r = new RobotController(this);
        r.aRamp = 1000.0;
        r.InitAndWait();
        // Moves forward to the stones
        r.DriveDistance(0,50, 0.24);
        // Grabs a skystone
        int skystone = GrabSkystone(true);
        // Drives to the building area
        r.DriveDistance(0,60, 0.24);
        // Lifts the extender
        r.manageExtender.enterNext();
        r.DriveDistance(0,40, 0.24);
        r.DriveRotate(0);
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        r.DriveDistance(0,40,0.24);
        r.DriveStop();
        r.hooksDown();
        r.manageExtender.enterNext();
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        r.manageExtender.enterNext();
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        r.DriveDistance(0, -60, 0.24);
        
        r.DriveRotate(-90);
        r.hooksUp();;
        r.DriveDistance(0, -10, 0.24);
        r.DriveStop();
        r.manageExtender.enterNext();
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        
        r.DriveDistance(0, -40+skystone*r.LStone, 0.24);
        r.manageExtender.enterNext();
        r.DriveDistance(0, -30+skystone*r.LStone, 0.24);
        r.DriveRotate(0);
        // Grabs a second stone
        GrabStone();
        r.DriveRotate(-90);
        r.DriveDistance(0,60, 0.24);
        r.DriveStop();
        // Lifts the extender
        r.manageExtender.enterNext();
        r.DriveDistance(0,60, 0.24);
        // placestone on the foundation
        r.manageExtender.enterNext();
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        r.manageExtender.enterNext();
        while (opModeIsActive() && r.manageExtender.running()) {
            r.BasicLoopTele();
        } 
        r.DriveDistance(0,-10, 0.24);
        r.manageExtender.enterNext();
        r.DriveDistance(0,-50, 0.24);
        

    }
}