package org.firstinspires.ftc.teamcode.TeleOperated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NonOpModes.HallSensor;
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
@TeleOp(name = "HallSensorTest", group = "Tests")
public class HallSensorTest extends LinearOpMode
{
    private HallSensor hallSensor[] = {null,null};
    BNO055IMU imu;
    public void runOpMode() throws InterruptedException
    {
        int counter=0;
        
        hallSensor[0] = hardwareMap.get(HallSensor.class, "HallSensor1");
        hallSensor[1] = hardwareMap.get(HallSensor.class, "HallSensor2");
        
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
        hallSensor[0].clrErr();
        hallSensor[1].clrErr();
 

        int L = 152, W = 70, r = 29;
        hallSensor[0].updateAngleRaw();
        hallSensor[1].updateAngleRaw();
        double x=0,y=0,theta=0, prevphi1 = hallSensor[0].getAngle(), prevphi2 = hallSensor[1].getAngle(), 
        prevtheta = 0,
        prevt = 0;
        ElapsedTime runtime = new ElapsedTime();
        while(opModeIsActive())
        {
            //counter++;
            for (int i =0; i<2; i++) {
                hallSensor[i].updateAngleRaw();
                telemetry.addData("Sensor", i);
                telemetry.addData("Angle", hallSensor[i].getAngle());
                //telemetry.addData("Field", hallSensor[i].getField());
                //telemetry.addData("errorcode=", hallSensor[i].getErr());
                //telemetry.addData("errorstr=", hallSensor[i].getErrStr());
            }
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = (angles.firstAngle);
            //telemetry.addData("Heading", heading);
            //telemetry.addData("counter=", counter);
            
            double dtheta = heading - prevtheta;
            if (Math.abs(dtheta) > Math.PI) {
                dtheta -= Math.PI * 2 * Math.signum(dtheta);
            }
            
            double dphi1 = hallSensor[0].getAngle()-prevphi1; 
            double dphi2 = hallSensor[1].getAngle()-prevphi2; 
            
            double dx = dphi1 * r - dtheta * W;
            double dy = dphi2 * r - dtheta * L;
            
            x += Math.cos(heading) * dx - Math.sin(heading)*dy;
            y += Math.sin(heading) * dx + Math.cos(heading)*dy;
            prevphi1 = hallSensor[0].getAngle();
            prevphi2 = hallSensor[1].getAngle();
            prevtheta = heading;
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            double t = runtime.time();
            telemetry.addData("dt", t-prevt);
            prevt = t;
            telemetry.update();
            // }
            idle();
        }
    }
}
