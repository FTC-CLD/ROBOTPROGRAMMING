package org.firstinspires.ftc.teamcode.NonOpModes;


public class FollowLine {
    public double radius;
    public double Cx, Cy, dividerBig, divider, A_x, A_y, magnitude, kP;
    public FollowLine(double Ax, double Ay, double Bx, double By, double P) {
        kP = P;
        Cx = Bx-Ax;
        Cy = By-Ay;
        magnitude = new Point(Cx,Cy).magnitude();
        // doe alvast 1/divider, zodat je niet nog een keer hoeft te delen
        dividerBig = 1/Math.sqrt((Cx)*(Cx)+(Cy)*(Cy));
        divider = ((Cx)*(Cx)+(Cy)*(Cy));
    }
    public double afstandLine(double Rx, double Ry){
        return ((Ry - A_y)*Cx - (Rx - A_x)*Cy)*dividerBig;
    }
    
    public boolean isFinished(double Rx, double Ry) {
        return ((Rx - A_x)*Cx + (Ry - A_y)*Cy >= divider);
    }
    
    public Point driveVector(double Rx, double Ry) {
        double x = (Cx - afstandLine(Rx,Ry)*Cy*kP)/magnitude;
        double y = (Cy + afstandLine(Rx,Ry)*Cx*kP)/magnitude;
        return new Point(x,y);
    }
    
}
