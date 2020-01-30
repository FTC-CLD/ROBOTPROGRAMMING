
package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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
@Disabled
@TeleOp(name="digitalInput", group="Linear Opmode")

public class digitalInput extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static DcMotor LT = null;
    public static DcMotor RT = null;
    public static  DcMotor LB = null;
    public static DcMotor RB = null;
    public static  Servo GripBlock = null;
    public static CRServo Extender = null; 
    public static AnalogInput GripButton;
    public static double LT_Power;
    public static double LB_Power;
    public static double RT_Power;
    public static double RB_Power;
    public static double Kracht;
    public static int[] forward = {1,1,1,1};
    public static int[] right = {-1,1,1,-1};
    public static boolean bPressed = false, Grip = false;

    
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LT  = hardwareMap.get(DcMotor.class, "lefttopmotor");

        GripButton = hardwareMap.get(AnalogInput.class, "GripTouch");
        //GripButton.setMode(DigitalChannel.Mode.INPUT);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            

            telemetry.addData("Analog","Button not Pressed"+String.valueOf(GripButton.getVoltage()));

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Linksvoor (%.2f), Rechtsvoor (%.2f), Linksachter (%.2f), Rechtsachter (%.2f)", LT_Power, RT_Power, LB_Power, RB_Power);
            telemetry.update();
        }
    }
}

