
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */



public class RobotController extends GyroAbsolute{
    // Declare OpMode members.
    public ElapsedTime runtime;
    private LinearOpMode opMode;
    public DcMotor[] driveMotors = {null,null,null,null};
    public static  Servo GripBlock = null, Capstone = null, Pusher = null, Foundation1, Foundation2;
    public static DcMotor extender = null; 
    public static Servo pusher = null;
    public static AnalogInput GripButton;       
    public static int timer = 0;
    public static double aRamp = 2.4, dt = 0.01, prevtime = -0.01;
    final public static double[] forward = {1,1,1,1}, right = {-1,1,0.9,-0.9}, turnclock = {-1,1,-1,1};
    public static double[] prevpower = {0,0,0,0};
    public static boolean gripPressed = false, foundationPressed = false, Grip = false, IsUp = false, IsDown = true, 
    extenderUp = true, foundationDown;
    
    // Extender 
    final public static double kG = 0.092, kF = 0.108, posToY = 57.0/820.0;

    
    public RobotController(LinearOpMode opModeIn) {
        opMode = opModeIn;
    }
    
    public double getK(double position,double velocity) {
        if (velocity>0) {
           return position*(0.26-0.190)/54+0.2;
        }
        else {
           return (position>25)?(-0.01+position*(0.26-0.190)/54):-0.013; 
        }
    }
    
    public double getExtenderPosition() {
        return extender.getCurrentPosition()*posToY;
    }
    // Function that takes x, y relative speeds as input and maps it to the power of the different wheel motors
    public void DriveSimple(double x, double y, double turn, double speed){
        assert speed >= 0;
        double maximum = 1;
        double[] power = {0,0,0,0};
        if (turn > 1) turn = 1;
        if (turn < -1) turn = -1;
        speed = Math.min(speed,1);
        for (int i = 0; i < 4; i++) {
            // The formula for the powers is given by y*forward+x*right+turn*turnclock
            // forward, right and turnclock are vectors that represent the max speed in their respective direction
            power[i] = (y * forward[i] + x * right[i])*Math.min(speed,1);
            double total = power[i] + turnclock[i] * turn;
            if (Math.abs(total) > 1) {
                maximum = Math.max(Math.abs(power[i]/(Math.signum(power[i])-turn*turnclock[i])),maximum);
            }
        }

        // If the maximum power exceeds the allowed power, we divide by this constant

        for (int i = 0; i < 4; i++) {
            power[i] /= maximum;
            power[i] += turn * turnclock[i];
        }
        
        
        for (int i=0;i<4;i++) {
            // Calculates what the delta v would be when target power is applied
            
            double dv = (power[i]-prevpower[i]);
            // If this dv is too big, get the max allowed acceleration*delta time and add it.
            if (dv > aRamp*dt) {
                power[i] = prevpower[i]+aRamp*dt;
            }
            // The same for negative dv
            else if (dv < -1*aRamp*dt) {
                power[i] = prevpower[i] - aRamp*dt;
            }
        }

        
        for (int i = 0; i < 4; i++) {
            //Sets the power of the wheels
            driveMotors[i].setPower(power[i]);
            //Copy power to prevpower
            prevpower[i] = power[i];
        }
    }
    
    
    public void Init() {
        opMode.telemetry.addData("Status", "Initialized");
        opMode.telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        driveMotors[0] = opMode.hardwareMap.get(DcMotor.class, "lefttopmotor");
        driveMotors[1] = opMode.hardwareMap.get(DcMotor.class, "righttopmotor");
        driveMotors[2] = opMode.hardwareMap.get(DcMotor.class, "leftbottommotor");
        driveMotors[3] = opMode.hardwareMap.get(DcMotor.class, "rightbottommotor");
        for (int i=0;i<4;i++) {
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            prevpower[i]= 0;
        }
        // The extender is run with handcrafted voltage controller
        extender = opMode.hardwareMap.get(DcMotor.class, "Extender");
        extender.setMode(DcMotor.RunMode.RESET_ENCODERS);
        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        extender.setDirection(DcMotor.Direction.REVERSE);
        
        GripBlock = opMode.hardwareMap.get(Servo.class, "GripBlock");
        Capstone = opMode.hardwareMap.get(Servo.class, "Capstone");
        
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        driveMotors[0].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[1].setDirection(DcMotor.Direction.FORWARD);
        driveMotors[2].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[3].setDirection(DcMotor.Direction.FORWARD);
        dt = 0.01;
        prevtime = -0.01;
        IsUp = false;
        IsDown = false;
        runtime = new ElapsedTime();

    }
    public void InitAndWait() {
        Init();
        opMode.waitForStart();
        runtime = new ElapsedTime();
    }
    public void resetRuntime() {
        runtime = new ElapsedTime();
    }
    public void BasicLoopTele() {
            dt = (runtime.time() - prevtime);
            prevtime = runtime.time();
            opMode.telemetry.update();
            timer++;
    }
    
}


