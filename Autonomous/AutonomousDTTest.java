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
@Autonomous(name="TestDt", group="Linear Opmode")
//@Disabled
public class AutonomousDTTest extends LinearOpMode  {
    public RobotController r;
    @Override
    public void runOpMode() {
        // Support class for robot control
        r = new RobotController(this);
        r.aRamp = 1000.0;
        r.InitAutoWait();
        while (opModeIsActive()) {
            r.BasicLoopTele();
        }
        //r.log1.close();
        

        
    
        

    }
}
