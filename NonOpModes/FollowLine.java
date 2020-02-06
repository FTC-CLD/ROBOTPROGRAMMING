package org.firstinspires.ftc.teamcode.NonOpModes;
import org.firstinspires.ftc.teamcode.NonOpModes.Point;
import org.firstinspires.ftc.teamcode.NonOpModes.PIDController;

public class FollowLine {
    public double Cx, Cy, dividerBig, divider, A_x, A_y, kP;
    public boolean klaar;
    private PIDController pid;
    double ensureRange(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
    
    public FollowLine(double Ax, double Ay, double Bx, double By, double skip) {
        Cx = Bx-Ax;
        Cy = By-Ay;
        A_x = Ax;
        A_y = Ay;
        // doe alvast 1/C^2, zodat je niet nog een keer hoeft te delen
        double distance = Math.sqrt(Cx*Cx+Cy*Cy);
        dividerBig = 1/distance;
        divider = 1/((distance-skip)*distance);
        final double Ku = 0.1, Tu = 0.98;
        pid = new PIDController(Ku/5.0,0.3*Ku/Tu,Ku*Tu/15);
        pid.setInputRange(0,2);
        pid.enable();
    }
    public double afstandLine(double Rx, double Ry){
        return ((Ry - A_y)*Cx - (Rx - A_x)*Cy)*dividerBig;
    }
    //returned waarde tussen 0 en 1(klaar)
    public double distanceDriven(double Rx, double Ry){
        return ensureRange(((Rx - A_x)*Cx + (Ry - A_y)*Cy)*divider,0,1);
    }
    public boolean isFinished(double Rx, double Ry) {
        return (((Rx - A_x)*Cx + (Ry - A_y)*Cy)*divider >=1);
    }
    // return x,y,hoek
    public Point Drive(double Rx, double Ry, double speed) {
        
        

        double error = afstandLine(Rx,Ry);
        double correction = -pid.performPID(error);
        // simpele P volger, + constante snelheid vooruit
        double x = (Cx + correction*Cy/speed)*dividerBig;
        double y = (Cy - correction*Cx/speed)*dividerBig;
        //returned x en y absoluut
        return new Point(x,y);
    }
    
}
