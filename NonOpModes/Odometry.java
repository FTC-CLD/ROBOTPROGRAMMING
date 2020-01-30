package org.firstinspires.ftc.teamcode.NonOpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NonOpModes.HallSensor;
import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;
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
public class Odometry
{
    final double degToCm = 2.8725;
    private RobotController r;
    private double x, y;
    final double xoffset = 4, yoffset = 15;
    double prevtheta = 0, prevphi1, prevphi2;
    public Odometry(RobotController R) {

        r = R;
        x=-xoffset+r.x;
        y=-yoffset+r.y;
        r.hallSensor[0].updateAngleRaw();
        r.hallSensor[1].updateAngleRaw();
        prevphi1 = r.hallSensor[0].getAngle();
        prevphi2 = r.hallSensor[1].getAngle();
    }
    public void update(double heading)
    {
            for (int i =0; i<2; i++) {
                r.hallSensor[i].updateAngleRaw();
                //telemetry.addData("Field", hallSensor[i].getField());
                //telemetry.addData("errorcode=", hallSensor[i].getErr());
                //telemetry.addData("errorstr=", hallSensor[i].getErrStr());
            }
            
            double dtheta = heading - prevtheta;
            
            
            double phi1 = r.hallSensor[0].getAngle();
            double phi2 = r.hallSensor[1].getAngle();
            
            double dphi1 = phi1 -prevphi1; 
            double dphi2 = phi2 -prevphi2; 
            
            ////

            
            double[] dP = deltaPos(heading,heading-prevtheta, dphi1*degToCm, -dphi2*degToCm);
             
            x += dP[0];
            y += dP[1];
            r.x = x+Math.cos(heading)*xoffset-Math.sin(heading)*yoffset;
            r.y = y+Math.sin(heading)*xoffset+Math.cos(heading)*yoffset;

            
            // dit is het laatste
            prevtheta = heading;
            prevphi1 = phi1;
            prevphi2 = phi2;
    }
    
    // om de positie te updaten met arcs
    public double[] deltaPos(double H,double deltaH, double encoder1, double encoder2){
        // H is de hoek van 1 frame geleden
        //integraal van een rotatie van deltaH met een verschuiving van encoder 2
        
        double cosH = Math.cos(H);
        double sinH = Math.sin(H);
        //als de hoek niet verandert is
        if (deltaH == 0){
            double output[] = {cosH*encoder2 - sinH*encoder1, cosH*encoder1 + sinH*encoder2};
            //return dx,dy
            return output;
        }
        
        double cosHnew = Math.cos(H + deltaH);
        double sinHnew = Math.sin(H + deltaH);
        
        //als de hoek van de robot wel verandert
        double dx = (encoder2*(sinHnew - sinH) + encoder1*(cosHnew - cosH))/deltaH;
        double dy = (encoder1*(sinHnew - sinH) - encoder2*(cosHnew - cosH))/deltaH;
        //return dx,dy
        return new double[] {dx,dy};
    }
}
