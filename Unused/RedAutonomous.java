
package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
 **
 ** FTC team 16531 'The Rolling Skystones'
 ** Copyright (c) 2019-2034
 **
 **/

@Autonomous(name="RedAutonomous", group="Linear Opmode")
@Disabled
public class RedAutonomous extends LinearOpMode {
    // Declare constants used by opMode members
    final double pusherCollecting = 0.0;
    final double pusherReturning = 1.0;
    final double gripblockClosing = 0.0;
    final double gripblockReleasing = 1.0;
    final double driverSpeedSlow = 0.25;
    final double driverSpeedMedium = 0.35;
    final double driverSpeedFast = 0.5;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static DcMotor LT = null;
    public static DcMotor RT = null;
    public static DcMotor LB = null;
    public static DcMotor RB = null;
    public static Servo GripBlock = null, Pusher = null, Foundation1 = null, Foundation2 = null;
    public static CRServo Extender = null; 
    public static AnalogInput GripButton;
    public static int timer = 0;
    public static double cmToEncode = 1000.0/58.0, degreesToEncode = 1000.0/102.0, rampingC = 0.9, dt = 0.01, prevtime = -0.01;
    public static int[] forward = {1,1,1,1}, right = {1,-1,-1,1}, turnclock = {-1,1,-1,1};
    public static double[] power = {0,0,0,0};//, prevpower = {0,0,0,0};
    public static boolean Grip = false, IsUp = false, IsDown = false, extenderUp = false;

    
    //check if the arm is extended, return True if extended
    public boolean Extended() {
        return (GripButton.getVoltage() > 0.5);
    }
    public void extend() {
        if (extenderUp) {
            extendingUp();
        }
        else {
            extendingDown();
        }
    }
    // Extend the arm up
    public void extendingUp() {
        if (Extended()) {
            if(IsDown) {
                Extender.setPower(-0.8);
            }
            else {
                IsUp = true;
                IsDown = false;
                Extender.setPower(0.0); 
            }
        } else {
            Extender.setPower(-0.8);
            IsDown = false;
            IsUp = false;
        }   
    }
    
    //Extend the arm down
    public void extendingDown() {
        if (Extended()) {
            if(IsUp) {
                Extender.setPower(0.8);
            }
            else {
                IsDown=true;
                IsUp = false;
                Extender.setPower(0.0); 
            }
        } else {
            Extender.setPower(0.8);
            IsDown = false;
            IsUp = false;

        }
    }

    public void grabBlock() {
        //Pusher.setPosition(0.0);
        Pusher.setPosition(pusherCollecting);
        sleep(1000);
        //GripBlock.setPosition(0.0);
        GripBlock.setPosition(gripblockClosing);
        sleep(200);
        //Pusher.setPosition(1.0);
        Pusher.setPosition(pusherReturning);

    }
    
    public void releaseBlock() {
        // GripBlock.setPosition(0.0); 
        GripBlock.setPosition(gripblockReleasing);
        sleep(200);

    }   
    
    public void setFoundationPosition(double position) {
        Foundation1.setPosition((1-position*0.65));
        Foundation2.setPosition(position);
    }
    
    public int[] GetWheelTurns(int x, int y, int turn) {
        int[] distances = {0,0,0,0};
        for (int i = 0; i < 4; i++) {
            distances[i] = (y + x * right[i] + turnclock[i]*turn);
        }
        return distances;
    }
    
    // Check if the robot is driving
    public boolean IsDriving(int n) {
        switch(n) {
            case 0:
                return LT.isBusy();
            case 1:
                return RT.isBusy();
            case 2:
                return LB.isBusy();
            default:
                return RB.isBusy();
        }

    }
    
    // Print an arrayDrive(1,1000,0.3);
    public void printarray(double[] a) {
        for (int i =0; i < 4; i++) {
           telemetry.addData("Position = "+String.valueOf(i), String.valueOf(a[i]) ) ;
        }
    }
    
    public void printarray(int[] a) {
        for (int i =0; i < 4; i++) {
           telemetry.addData("Position = "+String.valueOf(i), String.valueOf(a[i]) ) ;
        }
    
    }
    
    //
    // Function to drive Forward, Backward, Right or Left. 
    // Direction = 1 for Forward/Backward
    // Direction = 2 for Right/Left
    //
    
    public void Drive(int direction, int distance, double speed){
            switch(direction){
                case 1:
                    telemetry.addData("forward",' ');
                    telemetry.update();
                    // Forward/Backward
                    
                    for (int i = 0; i < 4; i++) {
                        power[i] = forward[i] * speed;
                    }
                    break;
                case 2:
                    telemetry.addData("side",' ');
                    telemetry.update();
                    // Right/Left
                    for (int i = 0; i < 4; i++) {
                        power[i] = right[i] * speed;
                    }
                    break;
            }
            
            // Reset the encodersimport java.util.concurrent.TimeUnit;
            LT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Set target position
            LT.setTargetPosition(distance);
            LB.setTargetPosition(distance);
            RT.setTargetPosition(distance);
            RB.setTargetPosition(distance);
            // Set motor mode
            LT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set motor power
            LT.setPower(power[0]);
            LB.setPower(power[1]);
            RT.setPower(power[2]);
            RB.setPower(power[3]);
            
            // Wait until the distance is travelled 
            while (opModeIsActive() && IsDriving(0)){
                telemetry.addData("Pos LT:", LT.getCurrentPosition());
                telemetry.addData("Pos RT:", RT.getCurrentPosition());
                telemetry.addData("Pos LB:", LB.getCurrentPosition());
                telemetry.addData("Pos RB:", RB.getCurrentPosition());
                //telemetry.update();
                idle();
            }
            
            
            // set motor power to zero to turn off motors. The motors stop on their own but
            // power is still applied so we turn off the power.
    }
    
    // Drive to x,y in cm and with a turn of turn degrees. Speed is between 0 and 1
    public void DriveDistance(double x, double y, int turn, double speed){
        int xencode = (int) Math.round(x*cmToEncode);
        int yencode = (int) Math.round(y*cmToEncode);
        int turnencode = (int) Math.round(turn*degreesToEncode);
        // Calculate the distance each wheel travels
        int[] wheelDistances = GetWheelTurns(xencode,yencode, turnencode);
        
        double magnitude = Math.sqrt(xencode*xencode+yencode*yencode)+turnencode;


        double jsleft_x = xencode/magnitude*speed;
        double jsleft_y = yencode/magnitude*speed;
        double turnpower = turnencode/magnitude*speed;
        double[] power = {0,0,0,0};
        double maximum = 0;
        for (int i = 0; i < 4; i++) {
            power[i] = (jsleft_y + jsleft_x * right[i]+ turnpower*turnclock[i]);
            maximum = Math.max(power[i],maximum);
        }
        if (maximum > 1){
           for (int i = 0; i < 4; i++) {
            power[i] /= maximum;
            }
        }
        printarray(wheelDistances);
        printarray(power);
        telemetry.update();
        // Reset the encoders
        LT.setMode(DcMotor.RunMode.RESET_ENCODERS);
        LB.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RT.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RB.setMode(DcMotor.RunMode.RESET_ENCODERS);
        // Set The target Position
        LT.setTargetPosition(wheelDistances[0]);
        RT.setTargetPosition(wheelDistances[1]);
        LB.setTargetPosition(wheelDistances[2]);
        RB.setTargetPosition(wheelDistances[3]);
        
        LT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Set The Power of the motors
        LT.setPower(power[0]);
        RT.setPower(power[1]);
        LB.setPower(power[2]);
        RB.setPower(power[3]);
        
        
        // Check which wheel moves the most distance;
        int maxDist = 0;
        int maxIndex = 0;
        for (int i=0; i<4; i++) {
            if(Math.abs(wheelDistances[i])> maxDist) {
                maxIndex = i;
                maxDist = Math.abs(wheelDistances[i]);
            }
        }
        while (opModeIsActive() && IsDriving(maxIndex)) {
                telemetry.addData("Pos LT:", LT.getCurrentPosition());
                telemetry.addData("Pos RT:", RT.getCurrentPosition());
                telemetry.addData("Pos LB:", LB.getCurrentPosition());
                telemetry.addData("Pos RB:", RB.getCurrentPosition());
                telemetry.update();
                //Optional parallel commands here;
                extend();
                idle();
        }
    }
    
    public void DriveStop() {
        LT.setPower(0);
        RT.setPower(0);
        LB.setPower(0);
        RB.setPower(0); 
    }
 
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        IsUp = false;
        IsDown = false;
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
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LT.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RT.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);
        GripButton = hardwareMap.get(AnalogInput.class, "GripTouch");
        GripBlock = hardwareMap.get(Servo.class, "GripBlock");
        Extender = hardwareMap.get(CRServo.class, "Extender");
        Pusher = hardwareMap.get(Servo.class, "Pusher");
        Foundation1 = hardwareMap.get(Servo.class, "Foundation1");
        Foundation2 = hardwareMap.get(Servo.class, "Foundation2");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        extenderUp = false;
        //Set the block pusher to right above the block and the gripper open
        Pusher.setPosition(0.5);
        GripBlock.setPosition(gripblockReleasing);
        setFoundationPosition(0.6);
        DriveDistance(0, 85, 0,driverSpeedSlow);
        // Get a stone from the 6 stones in the middle
        grabBlock();
        extenderUp = true;
        DriveDistance(0,-35,0,driverSpeedSlow);
        // Drives to other side under the bridge
        DriveDistance(200,0,0,driverSpeedSlow);
        DriveDistance(0, 37, 0,driverSpeedSlow);
        //Grabs the Foundation and releases the block unto the foundation
        setFoundationPosition(0.0);
        GripBlock.setPosition(gripblockReleasing);
        sleep(200);
        DriveDistance(0, -84, 0,driverSpeedSlow);

        // rij een tijdje tegen de muur aan
        LT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LT.setPower(-0.4);
        LB.setPower(-0.4);
        RT.setPower(-0.4);
        RB.setPower(-0.4);
        sleep(1000);
        LT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //Maakt ruimte voor zichzelf
        DriveDistance(0, 3, 0,driverSpeedSlow);
        //Releases the foundation
        setFoundationPosition(0.6);
        sleep(200);
        //Drives under the bridge to earn points
        extenderUp = false;
        DriveDistance(0, -1, 0,driverSpeedSlow);
        DriveDistance(-80, 0, 0,driverSpeedSlow);
        DriveDistance(0,50,0,driverSpeedSlow);
        DriveDistance(20,0,0,driverSpeedSlow);
        DriveDistance(-60,0,0,driverSpeedFast);
        
        // while(!IsUp) {
        //     extendingUp();
        // }

        
        DriveStop();
        //while(true) {}(0, 10000, 0, 0.4
            //Servos();

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Buttons","A" + String.valueOf(gamepad1.a)+"B"+String.valueOf(gamepad1.b));
            //telemetry.addData("Status", "Run Time: " + String.valueOf(timer/runtime.time()) + ' ' + String.valueOf(timer));
            //telemetry.addData("Motors", "Linksvoor (%.2f), Rechtsvoor (%.2f), Linksachter (%.2f), Rechtsachter (%.2f)", power[0], power[1],power[2],power[3]);

        
    }
}

