package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NonOpModes.LedControl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Created by Jeroen Op de River
 *
 * 
 */
@Disabled
@TeleOp(name = "LedControlTest", group = "Tests")
public class LedControlTest extends LinearOpMode
{
    private LedControl ledControl= null;
    BNO055IMU imu;
    public void runOpMode() throws InterruptedException
    {
        int counter=0;
        
        ledControl = hardwareMap.get(LedControl.class, "LedControl");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        

        
        while(opModeIsActive())
        {
            //counter++;
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            
            double heading = (angles.firstAngle);
            
            // heading+= 180;
            // telemetry.addData("Heading", heading);
            // heading *=12/360.0;
            // for (int i=0;i<12;i++) {
            //     int temp = Math.max((int)Math.round(60*(1.0-Math.abs(heading-i))),0);
            //     telemetry.addData("Led", temp);
            //     telemetry.addData("AndereKleur", (int)Math.round(0.6*(100-temp)));
            //     ledControl.setLed(i, temp,(int)Math.round(0.3*(100-temp)),(int)Math.round(0.3*temp));
            // }
            heading+=180;
            telemetry.addData("Heading", heading);
            ledControl.setCompass((int)Math.round(heading));
            
            sleep(100);
            telemetry.addData("counter=", counter);
            
            telemetry.update();
            // }
            idle();
        }
    }
}
