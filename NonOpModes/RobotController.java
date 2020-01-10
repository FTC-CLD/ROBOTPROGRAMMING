
package org.firstinspires.ftc.teamcode.NonOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */



public class RobotController {
    // Declare OpMode members.
    public ElapsedTime runtime;
    private LinearOpMode opMode;
    public DcMotor[] driveMotors = {null,null,null,null};
    public static  Servo GripBlock = null, Capstone = null, Pusher = null, Foundation1, Foundation2;
    public static DcMotor extender = null; 
    public static Servo pusher = null;
    public static AnalogInput GripButton;   
    public DistanceSensor distanceSensor;
    public ColorSensor sensorColor;
    public PIDController robotAngle;
    public double targetangle = 0;
    BNO055IMU imu;
    public double x,  y, heading;
    public double rx, ry, r, hr;
    
    public static int timer = 0;
    public static double aRamp = 2.4, dt = 0.01, prevtime = -0.01;
    final public static double[] forward = {1,1,1,1}, right = {-1,1,0.9,-0.9}, turnclock = {-1,1,-1,1};
    public static double[] prevpower = {0,0,0,0};
    public static boolean gripPressed = false, foundationPressed = false, Grip = false, IsUp = false, IsDown = true, 
    extenderUp = true, foundationDown;
    
    // Extender 
    final public static double kG = 0.092, kF = 0.108, posToY = 57.0/820.0, kV = 0.011, kA = 0.004;

    
    public RobotController(LinearOpMode opModeIn) {
        opMode = opModeIn;
    }
    
    
    
    // functie om simpel (zonder smooth p.) te rijden in de autonomous en bij het plaatsen van de capstone
    public void driveAutoInit(double targetx, double targety, double angle, double r, double hoekr){
        heading = heading();
        targetangle = angle;
        hr = hoekr;
        double distancetotarget = Math.sqrt((x-targetx)*(x-targetx)+(y-targety)*(y-targety));
        rx = targetx + r*(targetx-x)/distancetotarget;
        ry = targety + r*(targety-y)/distancetotarget;
        
    }
    
    public boolean driveFinished() {
        return opMode.opModeIsActive() && ((x-rx)*(x-rx)+(y-ry)*(y-ry) > r*r || Math.abs(heading-targetangle) < hr);
    }
    
    public boolean IsSkystone() {
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        if (Double.isNaN(distance)){
            return true;
        }
        double temp = (1.0*sensorColor.red())/sensorColor.blue();
        opMode.telemetry.addData("Skystone", temp);
        return temp < 1.25;//*(distance*distance)
    } 
    
    
    public void resetExtender() {
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        if (distance < 10) {
            extender.setPower(0.2);
            do {
                distance = distanceSensor.getDistance(DistanceUnit.CM);
            } while (opMode.opModeIsActive() && distance < 10);
        }
        extender.setPower(-0.2);
        do {
            distance = distanceSensor.getDistance(DistanceUnit.CM);
            opMode.telemetry.addData("distance sensor", distance);
            opMode.telemetry.addData("distance sensor limit", distanceSensor.distanceOutOfRange);
            opMode.telemetry.update();
        } while (opMode.opModeIsActive() && (Double.isNaN(distance) || distance > 10));
        extender.setPower(0);
        extender.setMode(DcMotor.RunMode.RESET_ENCODERS);
        extender.setPower(0);
        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        extender.setPower(0);
    }
    
    public double getExtenderPosition() {
        return extender.getCurrentPosition()*posToY+2.4;
    }
    
    public void DriveGyro() {
        heading = heading();
        // Calculates the robot X and robot Y velocity with respect to its heading
        double robotX = opMode.gamepad1.left_stick_x*Math.cos(heading) - opMode.gamepad1.left_stick_y*Math.sin(heading);
        double robotY = opMode.gamepad1.left_stick_x*Math.sin(heading) + opMode.gamepad1.left_stick_y*Math.cos(heading);
        
        // calculate how far the joystick is from its centre position
        double distance = distance(opMode.gamepad1.right_stick_x, opMode.gamepad1.right_stick_y);
        double turn = 0;
        if (distance != 0) {
          //targetangle = Math.atan2(opMode.gamepad1.right_stick_x , -opMode.gamepad1.right_stick_y);
          targetangle -= opMode.gamepad1.right_stick_x*0.5*dt;
        } 
        double error = -angleDifference(targetangle, heading);
        
        turn = robotAngle.performPID(error);
        // Drives the robot with these calculated values
        DriveSimple(robotX, robotY, turn, 0.18+opMode.gamepad1.right_trigger*0.5);
          
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
        
        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "sensorColor");
        sensorColor = opMode.hardwareMap.get(ColorSensor.class, "sensorColor");
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        driveMotors[0].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[1].setDirection(DcMotor.Direction.FORWARD);
        driveMotors[2].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[3].setDirection(DcMotor.Direction.FORWARD);
        
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        targetangle = 0;
        
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
        int Ku = 3;
        double Tu = 0.45;
        robotAngle = new PIDController(Ku*0.6, 0.2*Ku/Tu,3*Ku*Tu/40);
        robotAngle.setInputRange(0, 10000);
        robotAngle.setOutputRange(0, 0.2);
        robotAngle.enable(); 
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
    
    public double heading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double heading = (angles.firstAngle);
        return heading;
    }
    
    public double angleDifference(double heading, double targetAngle) {
        // Calculates the angle difference. Always between -180 and 180
        return ((targetAngle-heading+Math.PI) % (2*Math.PI) -  Math.PI);
    }
    
    public double distance(double x,double y) {
        return Math.sqrt(x*x+y*y);
    }
    
}


