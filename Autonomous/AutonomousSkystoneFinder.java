package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;
@Autonomous(name="SkystoneFinder", group="Linear Opmode")

public class AutonomousSkystoneFinder extends LinearOpMode  {
    public RobotController r;
    
    @Override
    public void runOpMode() {
        // Support class for robot control
        r = new RobotController(this);
        r.Init();

    
        
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            r.BasicLoopTele();
            telemetry.addData("IsSkystone",r.IsSkystone());
                        telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", r.distanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", r.sensorColor.alpha());
            telemetry.addData("Red  ", r.sensorColor.red());
            telemetry.addData("Green", r.sensorColor.green());
            telemetry.addData("Blue ", r.sensorColor.blue());
            telemetry.update();

    
            
        }
    }
    // todo: write your code here
}