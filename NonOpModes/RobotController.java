
package org.firstinspires.ftc.teamcode.NonOpModes;
import org.firstinspires.ftc.teamcode.NonOpModes.Extender;
import org.firstinspires.ftc.teamcode.NonOpModes.LogFile;
import org.firstinspires.ftc.teamcode.NonOpModes.HallSensor;
import org.firstinspires.ftc.teamcode.NonOpModes.Odometry;
import org.firstinspires.ftc.teamcode.NonOpModes.FollowLine;
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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import android.widget.TextView;
import android.widget.RelativeLayout;
import android.util.TypedValue;

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
    public LinearOpMode opMode;
    public DcMotor[] driveMotors = {null,null,null,null};
    public Servo GripBlock = null, Capstone = null, Foundation1, Foundation2;
    public DcMotor extender = null; 
    public Extender manageExtender;
    public DistanceSensor distanceSensor;
    public ColorSensor sensorColor;
    public PIDController robotAngle;
    public double targetangle = 0, endangle = 0;
    public Odometry odometry;
    public HallSensor hallSensor[] = {null,null};
    BNO055IMU imu;
    public double x=0,  y=0, heading, prevx, prevy;
    public double rx, ry, r, hr;
    
    public static int timer = 0;
    public double aRamp = 2.4, dt = 0.01, prevtime = -0.01;
    
    // Constants for the chassis
    final public static double[] forward = {0.9,0.9,0.9,0.9}, right = {-1,1,1,-1}, turnclock = {-1,1,-1,1};
    final public double cmToEncode = 20.97, degreesToEncode = 1000.0/102.0;
    
    public static double[] prevpower = {0,0,0,0};
    public static boolean gripPressed = false, foundationPressed = false, Grip = false, IsUp = false, IsDown = true, 
    extenderUp = true, foundationDown;
    
    // Extender 
    final public static double kG = 0.092, kF = 0.108, posToY = 57.0/820.0, kV = 0.011, kA = 0.004;
    final public double LStone = 20.3;
    
    public LogFile log1, log2;
    public TextView mTextView;
    RelativeLayout relativeLayout;
    public int stateScreen;
    public RobotController(LinearOpMode opModeIn) {
        opMode = opModeIn;
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
            extender.setPower(0.4);
            do {
                distance = distanceSensor.getDistance(DistanceUnit.CM);
            } while (opMode.opModeIsActive() && distance < 10);
        }
        extender.setPower(-0.15);
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
        return extender.getCurrentPosition()*posToY+3.3;
    }
    
    public void DriveGyro() {
        // Calculates the robot X and robot Y velocity with respect to its heading
        double robotX = - opMode.gamepad1.left_stick_x*Math.cos(heading) + opMode.gamepad1.left_stick_y*Math.sin(heading);
        double robotY = - opMode.gamepad1.left_stick_x*Math.sin(heading) - opMode.gamepad1.left_stick_y*Math.cos(heading);
        
        // calculate how far the joystick is from its centre position
        double distance = distance(opMode.gamepad1.right_stick_x, opMode.gamepad1.right_stick_y);
        double turn = 0;
        if (distance != 0) {
          //targetangle = Math.atan2(opMode.gamepad1.right_stick_x , -opMode.gamepad1.right_stick_y);
          targetangle -= opMode.gamepad1.right_stick_x*1.2*dt;
          targetangle %= Math.PI*2;
        } 
        double error = -angleDifference(targetangle, heading);
        
        turn = robotAngle.performPID(error);
        // Drives the robot with these calculated values
        DriveSimple(robotX, robotY, turn, 0.26+opMode.gamepad1.right_trigger*0.4+opMode.gamepad1.left_trigger*0.4);
          
    }
    
    // Function that takes x, y relative speeds as input and maps it to the power of the different wheel motors
    public void DriveSimple(double x, double y, double turn, double speed){
        assert speed >= 0;
        double maximum = 1;
        double[] power = {0,0,0,0};
        speed = Math.min(speed,1);
        for (int i = 0; i < 4; i++) {
            // The formula for the powers is given by y*forward+x*right
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
    
    public void placeCapstone() {
      if (manageExtender.state == Extender.ExtenderState.AwayDriving) {
          
      }
    }
    
    // Turns the robot to absolute angle 
    public void DriveRotate(double newangle) {

        DriveRotate(newangle, 0.2);
    }
        // Turns the robot to absolute angle
    public void DriveRotate(double newangle, double speed) {

        newangle = Math.toRadians(newangle);
        double sign = angleDifference(newangle, targetangle);
        targetangle = newangle;
        endangle = newangle;
        do {
            BasicLoopTele();
            DriveSimple(0,0,sign*speed,0.3);

        } while (opMode.opModeIsActive() && Math.abs(angleDifference(heading,newangle)) > 0.2);
        robotAngle.reset();
        
    }
    
    
    // Drive to x,y in cm and with Speed is between 0 and 1
    public void DriveDistance(double x, double y, double speed){
        // Reset the encoders
        for (int i =0; i< 4; i++) {
            driveMotors[i].setMode(DcMotor.RunMode.RESET_ENCODERS);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        }
        for (int i = 0;i<4;i++){
            prevpower[i] = 0;
        }
        double xencode = x*cmToEncode;
        double yencode = y*cmToEncode;
        double magnitude = Math.sqrt(x*x+y*y);
        // Calculate the distance each wheel travels
        double[] wheelDistances = GetWheelTurns(xencode, yencode);

        // sum all absolute distances
        double sumDist = 0;
        for (int i=0; i<4; i++) {
            sumDist+=Math.abs(wheelDistances[i]);
        }
        while (opMode.opModeIsActive() && IsDriving(sumDist)) {
            BasicLoopTele();
            double error = -angleDifference(targetangle, heading);
            double turn = robotAngle.performPID(error);
            DriveSimple(x/magnitude, y/magnitude, turn, speed);
            //Optional parallel commands here;
        }
    }
    
    //hetzelfde als drivedistance, maar met ramp
    public void DriveRamp(double x, double y, double speed1, double speed2){
        // Reset the encoders
        for (int i =0; i< 4; i++) {
            driveMotors[i].setMode(DcMotor.RunMode.RESET_ENCODERS);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        }
        for (int i = 0;i<4;i++){
            prevpower[i] = 0;
        }
        double xencode = x*cmToEncode;
        double yencode = y*cmToEncode;
        // heb ik net verandert naar xencode van x
        double magnitude = Math.sqrt(xencode*xencode+yencode*yencode);
        // Calculate the distance each wheel travels
        double[] wheelDistances = GetWheelTurns(xencode, yencode);

        // sum all distances
        double sumDist = 0;
        for (int i=0; i<4; i++) {
            sumDist+=Math.abs(wheelDistances[i]);
        }
        while (opMode.opModeIsActive() && IsDriving(sumDist)) {
            BasicLoopTele();
            int curPos = 0;
            for (int i=0;i<4;i++) {
                curPos+=Math.abs(driveMotors[i].getCurrentPosition());
            }
            // de power moet nog in de wielen geplugd worden
            double speedRamp = speed1 + Math.sqrt((double)(curPos)/sumDist)*(speed2-speed1);
            double error = -angleDifference(targetangle, heading);
            double turn = robotAngle.performPID(error);
            DriveSimple(xencode/magnitude, yencode/magnitude, turn, speedRamp);
            //Optional parallel commands here;
        }
    }
    
    
    public boolean IsDriving(double sumDist) {
        int curPos = 0;
        for (int i=0;i<4;i++) {
            curPos+=Math.abs(driveMotors[i].getCurrentPosition());
        }
        return (curPos<sumDist);
    }
    private void updateTargetAngle() {
        if (targetangle != endangle) {
                double sign = Math.signum(angleDifference(endangle, targetangle));
                boolean positive = sign>0;
                targetangle -= sign*dt;
                if (positive ^ angleDifference(endangle,targetangle)>0) {
                    targetangle = endangle;
                }
                targetangle %= Math.PI*2;
            }
    }
    // Drives with the help of odometry wheels to absolute position
    public void DriveEncode(double targetx, double targety, double speed, double skipDistance) {
        FollowLine follower = new FollowLine(prevx,prevy,targetx,targety, skipDistance);
        do {
            BasicLoopTele();
            updateTargetAngle();
            Point a = follower.Drive(x,y,speed);
            double robotX = -a.x*Math.cos(heading) - a.y*Math.sin(heading);
            double robotY = -a.x*Math.sin(heading) + a.y*Math.cos(heading);
            double error = -angleDifference(targetangle, heading);
            double turn = robotAngle.performPID(error);
            DriveSimple(robotX, robotY, turn,  speed);
        } while (opMode.opModeIsActive()&&!follower.isFinished(x,y));
        prevx = targetx;
        prevy = targety;
    }
    public void setEndAngle(double angle) {
        endangle = Math.toRadians(angle);
    }
    public void DriveEncodeRamp(double targetx, double targety, double speed1, double speed2, double skipDistance) {
        FollowLine follower = new FollowLine(prevx,prevy,targetx,targety,skipDistance);
        do {
            BasicLoopTele();
            updateTargetAngle();
            double speedRamp = speed1+(speed2-speed1)*Math.sqrt(follower.distanceDriven(x,y));
            Point a = follower.Drive(x,y,speedRamp);
            double robotX = -a.x*Math.cos(heading) - a.y*Math.sin(heading);
            double robotY = -a.x*Math.sin(heading) + a.y*Math.cos(heading);
            double error = -angleDifference(targetangle, heading);
            double turn = robotAngle.performPID(error);
            DriveSimple(robotX, robotY, turn, speedRamp);
        } while (opMode.opModeIsActive()&&!follower.isFinished(x,y));
        prevx = targetx;
        prevy = targety;
    }
    public void DriveStop() {
        for (int i=0;i<4;i++) {
            driveMotors[i].setPower(0);
        }
    
    }
    
    public void hooksDown() {
        Foundation1.setPosition(0.9);
        Foundation2.setPosition(0.1);
    }
    
    public void hooksUp() {
        Foundation1.setPosition(0);
        Foundation2.setPosition(1);
    }
    
    public double[] GetWheelTurns(double x, double y) {
        double[] distances = {0,0,0,0};
        for (int i = 0; i < 4; i++) {
            distances[i] = (y + x * right[i]);
        }
        return distances;
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
        Foundation1 = opMode.hardwareMap.get(Servo.class, "HookL");
        Foundation2 = opMode.hardwareMap.get(Servo.class, "HookR");
        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "sensorColor");
        sensorColor = opMode.hardwareMap.get(ColorSensor.class, "sensorColor");
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        driveMotors[0].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[1].setDirection(DcMotor.Direction.FORWARD);
        driveMotors[2].setDirection(DcMotor.Direction.REVERSE);
        driveMotors[3].setDirection(DcMotor.Direction.FORWARD);
        x=0;
        y=0;
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
    public void InitAuto() {
        Init();
        hallSensor[0] = opMode.hardwareMap.get(HallSensor.class, "HallSensor1");
        hallSensor[1] = opMode.hardwareMap.get(HallSensor.class, "HallSensor2");
        hallSensor[0].clrErr();
        hallSensor[1].clrErr();

        odometry = new Odometry(this);
    }
    public void InitAutoLog() {
        log1 = new LogFile();
        log1.init("loggingauto.txt");
        log1.write("time,x,y,theta\n");
    }
        
    private void InitWaitInside() {
        opMode.waitForStart();
        double Ku = 1.2;//2.1
        double Tu = 1.0;//0.7
        robotAngle = new PIDController(Ku*0.6, 0.2*Ku/Tu,3*Ku*Tu/40);
        robotAngle.setInputRange(0, 10000);
        robotAngle.setOutputRange(0, 0.4);
        robotAngle.enable();
        GripBlock.setPosition(1);
    }
    
    public void InitAutoWait() {
        InitAuto();
        InitWaitInside();
        resetExtender();
        runtime = new ElapsedTime();
        manageExtender = new Extender(this);
    }

    public void InitAndWait() {
        Init();
        InitWaitInside();
        resetExtender();
        // log2 = new LogFile();
        // log2.init("loggingext.txt");
        // log2.write("time,pos,targetpos\n");
        runtime = new ElapsedTime();
        manageExtender = new Extender(this);
    }
    public void InitExtend() {
        Init();
        InitWaitInside();
        runtime = new ElapsedTime();
        manageExtender = new Extender(this);
    }
    
    public void resetRuntime() {
        runtime = new ElapsedTime();
    }
    public void BasicLoopTele() {
            dt = (runtime.time() - prevtime);
            prevtime = runtime.time();
            opMode.telemetry.update();
            timer++;
            heading = getHeading();
            if (odometry != null) {
                odometry.update(heading);
            }
            if (log1 != null) {
                log1.write(String.valueOf(prevtime)+","+String.valueOf(x)+","+String.valueOf(y)+","+String.valueOf(heading)+"\n");
                log1.flush();
            }
            manageExtender.update();

            opMode.telemetry.addData("x", x);
            opMode.telemetry.addData("y", y);
            opMode.telemetry.addData("fps", 1/dt);
            opMode.telemetry.addData("dt", dt);
            opMode.telemetry.addData("state", manageExtender.state);
            opMode.telemetry.addData("Extender Position", manageExtender.position);
    }
    
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double heading = (angles.firstAngle);
        return heading;
    }
    
    public void initScreen() {
        int relativeLayoutId = opMode.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", opMode.hardwareMap.appContext.getPackageName());
        relativeLayout = (RelativeLayout)((Activity) opMode.hardwareMap.appContext).findViewById(relativeLayoutId);
        mTextView = new TextView(opMode.hardwareMap.appContext);
        relativeLayout.post(new Runnable() {
        public void run() {
        mTextView.setText("1");
        mTextView.setTextSize(TypedValue.COMPLEX_UNIT_SP,500);
        RelativeLayout.LayoutParams params = new RelativeLayout.LayoutParams(RelativeLayout.LayoutParams.WRAP_CONTENT, RelativeLayout.LayoutParams.WRAP_CONTENT);
        params.addRule(RelativeLayout.CENTER_IN_PARENT, RelativeLayout.TRUE);
  
          relativeLayout.addView(mTextView, params);
        }
    });
    }
    
    public void updateScreen() {
        stateScreen = manageExtender.state.ordinal()+1;
        relativeLayout.post(new Runnable() {
            public void run() {
               mTextView.setText(String.valueOf(stateScreen));
            }
        });
    }
    public void closeScreen() {
        
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.removeView(mTextView);
            }
        });

    }
    public double angleDifference(double heading, double targetAngle) {
        // Calculates the angle difference. Always between -180 and 180
        return ((targetAngle-heading+5*Math.PI) % (2*Math.PI)) -  Math.PI;
    }
    
    public double distance(double x,double y) {
        return Math.sqrt(x*x+y*y);
    }
    
}

