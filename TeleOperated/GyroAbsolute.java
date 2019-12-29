
package org.firstinspires.ftc.teamcode.TeleOperated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@TeleOp(name="GyroAbsolute", group="Linear Opmode")

public class GyroAbsolute extends LinearOpMode {
    private DcMotor extender;
    public RobotController robot;
    PIDController robotAngle;
    BNO055IMU imu;
    double targetangle = 0;
    
    public double angleDifference(double heading, double targetAngle) {
        // Calculates the angle difference. Always between -180 and 180
        return ((targetAngle-heading+Math.PI) % (2*Math.PI) -  Math.PI);
    }
    
    public double distance(double x,double y) {
        return Math.sqrt(x*x+y*y);
    }
    
    public void PUSHER(double value) {
        RobotController.pusher.setPosition(value);
    }
    
    
    @Override
    public void runOpMode() {
        // Support class for robot control
        robot = new RobotController(this);
        robot.Init();
        int Ku = 3;
        double Tu = 0.45;
        robotAngle = new PIDController(Ku*0.6, 0.2*Ku/Tu,3*Ku*Tu/40);
        robotAngle.setInputRange(0, 10000);
        robotAngle.setOutputRange(0, 0.2);
        robotAngle.enable();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        extender = hardwareMap.dcMotor.get("Extender");
        extender.setMode(DcMotor.RunMode.RESET_ENCODERS);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        targetangle = 0;
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            robot.BasicLoopTele();
        
            //Here is the code you will need for reading the heading of the IMU
            //you may have to adjust which of X, Y, and Z you are reading based on your orientation of the hub
            //you can use this wherever you need to use the heading value in teleop loop or inside an autonomous loop for turning for example
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = (angles.firstAngle);
            telemetry.addData("Heading", heading);
            heading = Math.toRadians(heading);
            
            // Calculates the robot X and robot Y velocity with respect to its heading
            double robotX = gamepad1.left_stick_x*Math.cos(heading) - gamepad1.left_stick_y*Math.sin(heading);
            double robotY = gamepad1.left_stick_x*Math.sin(heading) + gamepad1.left_stick_y*Math.cos(heading);
            
            // calculate how far the joystick is from its centre position
            double distance = distance(gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Distance", distance);
            double turn = 0;
            if (distance != 0) {
                targetangle = Math.atan2( -gamepad1.right_stick_x, -gamepad1.right_stick_y);
                telemetry.addData("targetAngle",targetangle );
                // Here a PID can improve the accuracy

            }
            double error = -angleDifference(targetangle, heading);
            telemetry.addData("error", error );
            turn = robotAngle.performPID(error);
            telemetry.addData("turnCorrection",turn );
            
            // Drives the robot with these calculated values
            robot.DriveSimple(robotX, robotY, turn, 0.18+gamepad1.right_trigger*0.82);
            
            
        double speed = gamepad2.left_stick_y*0.4;
        telemetry.addData("speed", speed);
        telemetry.addData("position",extender.getCurrentPosition());
        if ((extender.getCurrentPosition() >= 0 && speed > 0) || (extender.getCurrentPosition() <= -820 && speed < 0)) {
          speed = 0;
        }
        telemetry.addData("speed with correction", speed);
        extender.setPower(speed);
        robot.GripBlock.setPosition(1-gamepad2.left_trigger);
    
            
        }
    }
}


