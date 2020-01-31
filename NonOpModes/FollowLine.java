package org.firstinspires.ftc.teamcode.NonOpModes;
import org.firstinspires.ftc.teamcode.NonOpModes.Point;

public class FollowLine {
    public double Cx, Cy, dividerBig, divider, A_x, A_y, kP;
    public boolean klaar;
    
    public FollowLine(double Ax, double Ay, double Bx, double By, double P) {
        kP = P;
        Cx = Bx-Ax;
        Cy = By-Ay;
        A_x = Ax;
        A_y = Ay;
        // doe alvast 1/C^2, zodat je niet nog een keer hoeft te delen
        dividerBig = 1/Math.sqrt(Cx*Cx+Cy*Cy);
        divider = 1/(Cx*Cx+Cy*Cy);
    }
    public double afstandLine(double Rx, double Ry){
        return ((Ry - A_y)*Cx - (Rx - A_x)*Cy)*dividerBig;
    }
    //returned waarde tussen 0 en 1(klaar)
    public double distanceDriven(double Rx, double Ry){
        return ((Rx - A_x)*Cx + (Ry - A_y)*Cy)*divider;
    }
    public boolean isFinished(double Rx, double Ry) {
        return (((Rx - A_x)*Cx + (Ry - A_y)*Cy)*divider >=1);
    }
    // return x,y,hoek
    public Point Drive(double Rx, double Ry) {
        
        


        // simpele P volger, + constante snelheid vooruit
        double x = (Cx + afstandLine(Rx,Ry)*Cy*kP)*dividerBig;
        double y = (Cy - afstandLine(Rx,Ry)*Cx*kP)*dividerBig;
        //returned x en y absoluut
        return new Point(x,y);
    }
    
}
