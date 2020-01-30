
package org.firstinspires.ftc.teamcode.Unused;

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
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="MecanumTest", group="Linear Opmode")

public class FirstLeagueMeetTele extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static DcMotor LT = null;
    public static DcMotor RT = null;
    public static  DcMotor LB = null;
    public static DcMotor RB = null;
    public static  Servo GripBlock = null, Capstone = null, Pusher = null, Foundation1, Foundation2;
    public static CRServo Extender = null; 
    public static AnalogInput GripButton;       
    public static int timer = 0;
    public static double Kracht, aRamp = 2.4, dt = 0.01, prevtime = -0.01;
    public static int[] forward = {1,1,1,1}, right = {-1,1,1,-1}, turnclock = {-1,1,-1,1};
    public static double[] power = {0,0,0,0}, prevpower = {0,0,0,0};
    public static boolean bPressed = false, aPressed = false, Grip = false, IsUp = false, IsDown = true, goingUp = true, foundationDown;
    
    public boolean Extended() {
        return (GripButton.getVoltage() > 0.5);
    }
    
    public void extendUp() {
        telemetry.addData("Going Up","JAJA");
        if (Extended()) {
            if(IsDown) {
                Extender.setPower(-0.7);
            }
            else {
                IsUp = true;
                IsDown = false;
                Extender.setPower(0.0); 
            }
        } else {
            Extender.setPower(-0.7);
            IsDown = false;
            IsUp = false;

        }
    }
    
    public void extendDown() {
        telemetry.addData("Going Down","JAJA");
        if (Extended()) {
            if(IsUp) {
                Extender.setPower(0.7);
            }
            else {
                IsDown=true;
                IsUp = false;
                Extender.setPower(0.0); 
            }
        } else {
            Extender.setPower(0.7);
            IsDown = false;
            IsUp = false;

        }
    }
    
    public void UserInputExtender() {
        if( gamepad1.y){
            goingUp=true;
        } else if (gamepad1.x) {
            goingUp = false;
        }
        
        if(goingUp) {
            extendUp();
            
        }
        else {
            extendDown();
        }
    }
    
    public void UserInputGrip() {
        if (gamepad1.b && !bPressed) {
            if(GripBlock.getPosition() == 1) {
                GripBlock.setPosition(0);
            }
            else {
                GripBlock.setPosition(1);
            }
        }
        bPressed = gamepad1.b;
    }
    
    public void UserInputFoundation() {
        telemetry.addData("Foundationpos", Foundation2.getPosition());
        
        if (gamepad1.a && !aPressed) {
            if(Foundation2.getPosition() == 0) {
                Foundation1.setPosition(0.6);
                Foundation2.setPosition(0.6);
            }
            else {
                Foundation1.setPosition(1);
                Foundation2.setPosition(0);
            }
        }
        aPressed = gamepad1.a;
    }
    
    // Function that takes joystick input and maps it to the power of the different wheel motors
    public void Drive(){
        double jsleft_y = gamepad1.left_stick_y;
        double jsleft_x = gamepad1.left_stick_x;
        double turn  =  gamepad1.right_stick_x;
        double maximum = 0;
        for (int i = 0; i < 4; i++) {
            // The formula for the powers is given by y*forward+x*right+turn*turnclock
            // forward, right and turnclock are vectors that represent the max speed in their respective direction
            power[i] = (jsleft_y + jsleft_x * right[i] + turn * 0.7 * turnclock[i]);
            telemetry.addData("power", power[i]);
            maximum = Math.max(Math.abs(power[i]),maximum);
        }
        double correction = 1;
        if (gamepad1.left_bumper) correction = 0.3;
        telemetry.addData("leftBumper",String.valueOf(gamepad1.left_bumper));
        // If the maximum power exceeds the allowed power, we divide by this constant
        if (maximum > 1){
           for (int i = 0; i < 4; i++) {
            power[i] /= maximum;
            }
        }
        for (int i = 0; i < 4; i++) {
            power[i] *= correction;
            }
        
        
        for (int i=0;i<4;i++) {
            // Calculates what the delta v would be when target power is applied
            
            double dv = (power[i]-prevpower[i]);
            // dit kan zo weg
            //dt = 0.002;
            telemetry.addData("dv",dv); 
            // If this dv is too big, get the max allowed acceleration*delta time and add it.
            if (dv > aRamp*dt) {
                power[i] = prevpower[i]+aRamp*dt;
            }
            // The same for negative dv
            else if (dv < -1*aRamp*dt) {
                power[i] = prevpower[i] - aRamp*dt;
            }
        }

        //Sets the power of the wheels
        LT.setPower(power[0]);
        RT.setPower(power[1]);
        LB.setPower(power[2]);
        RB.setPower(power[3]);
        //Copy power to prevpower
        for (int i = 0; i < 4; i++) {
            prevpower[i] = power[i];
        }
    }
    
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LT  = hardwareMap.get(DcMotor.class, "lefttopmotor");
        RT = hardwareMap.get(DcMotor.class, "righttopmotor");
        LB = hardwareMap.get(DcMotor.class, "leftbottommotor");
        RB = hardwareMap.get(DcMotor.class, "rightbottommotor");
        LT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GripBlock = hardwareMap.get(Servo.class, "GripBlock");
        Capstone = hardwareMap.get(Servo.class, "Capstone");
        Pusher = hardwareMap.get(Servo.class, "Pusher");
        Extender = hardwareMap.get(CRServo.class, "Extender");
        Foundation1 = hardwareMap.get(Servo.class, "Foundation1");
        Foundation2 = hardwareMap.get(Servo.class, "Foundation2");
        


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LT.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RT.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);
        GripButton = hardwareMap.get(AnalogInput.class, "GripTouch");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        dt = 0.01;
        prevtime = -0.01;
        for (int i=0; i<4; i++) {
            prevpower[i]= 0;
        }
        IsUp = false;
        IsDown = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            dt = (runtime.time() - prevtime);
            prevtime = runtime.time();
            Drive();
           
            // UserInputExtender();
            UserInputFoundation();
            UserInputGrip();
            Capstone.setPosition(1-gamepad1.left_trigger);
            Pusher.setPosition(1-gamepad1.right_trigger);
            telemetry.addData("Left_Trigger",gamepad1.left_trigger);

            telemetry.update();
            timer++;
        }
    }
}
 // telemetry.addData("Pos LT:", LT.getCurrentPosition());
            // telemetry.addData("Pos RT:", RT.getCurrentPosition());
            // telemetry.addData("Pos LB:", LB.getCurrentPosition());
            // telemetry.addData("Pos RB:", RB.getCurrentPosition());

