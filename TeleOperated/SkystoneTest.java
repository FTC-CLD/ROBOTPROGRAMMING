
package org.firstinspires.ftc.teamcode.TeleOperated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.logging.Logger;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.lang.reflect.Array;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Arrays;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;
import org.firstinspires.ftc.teamcode.NonOpModes.PIDController;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SkystoneTest", group="Linear Opmode")

public class SkystoneTest extends LinearOpMode {
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
}


