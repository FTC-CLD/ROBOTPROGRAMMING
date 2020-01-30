package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;
import org.firstinspires.ftc.teamcode.NonOpModes.Extender;
@Disabled
@Autonomous(name="RampoeTest", group="Linear Opmode")

public class AutonomousRamp extends LinearOpMode  {
    public RobotController r;
    
    @Override
    public void runOpMode() {
        // Support class for robot control
        r = new RobotController(this);
        r.aRamp = 1000.0;
        r.InitAndWait();
        // Moves forward to the stones
        r.DriveRamp(0,70, 0.15, 1);
        r.DriveRamp(0,70, 1, 0.05);
        r.DriveRamp(0,-70, 0.15, 1);
        r.DriveRamp(0,-70, 1, 0.05);
        r.DriveStop();

    }
}
