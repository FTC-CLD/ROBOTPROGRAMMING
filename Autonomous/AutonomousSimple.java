package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;
import org.firstinspires.ftc.teamcode.NonOpModes.Extender;
@Autonomous(name="AutonomousSimple", group="Linear Opmode")

public class AutonomousSimple extends LinearOpMode  {
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
        r.DriveDistance(1.15*r.LStone*alliance,0, 0.4);
        if (r.IsSkystone()) {
            GrabStone();
            r.DriveRotate(-90*alliance);
            r.DriveDistance(0,r.LStone*alliance, 0.24);
            return 1;
        }
        r.DriveDistance(1.15*r.LStone*alliance, 0,0.4);
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
        r.manageExtender.enterNext();
        r.DriveDistance(0,-10,0.26);

    }
}
