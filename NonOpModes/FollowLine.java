package org.firstinspires.ftc.teamcode.NonOpModes;


public class FollowLine {
    public double radius;
    public double Cx, Cy, dividerBig, divider, A_x, A_y, AngleA, AngleB, dAngle, myRotateLeft, kP, angleArrived = false;
    public FollowLine(double Ax, double Ay, double Bx, double By, double myAngleA, double myAngleB, boolean myRotateLeft, double P) {
        kP = P;
        Cx = Bx-Ax;
        Cy = By-Ay;
        AngleA = myAngleA;
        AngleB = myAngleB;
        RotateLeft = myRotateLeft
        
        // Het verschil in hoek (AKA hoeveel de robot op moet draaien) in graden (positief = naar links, negatief naar rechts)
        dAngle = (AngleB-AngleA+380)%360;
        if (!RotateLeft){
            dAngle = dAngle - 360;
        }
        
        // doe alvast 1/divider, zodat je niet nog een keer hoeft te delen
        dividerBig = 1/Math.sqrt((Cx)*(Cx)+(Cy)*(Cy));
        divider = 1/((Cx)*(Cx)+(Cy)*(Cy));
        
    }
    
    //returned de afstand van de robot tot de denkbeeldige lijn, in units of choise (cm)
    public double afstandLine(double Rx, double Ry){
        return ((Ry - A_y)*Cx - (Rx - A_x)*Cy)*dividerBig;
    }
    
    //returned hoe ver je al hebt gereden (0=min, 1 =max)
    public double fractionTraveled(double Rx, double Ry){
        return ((Rx - A_x)*Cx + (Ry - A_y)*Cy)*divider;
    }
    //hetzelfde als hierboven, maar checked of je al ver genoeg hebt gereden
    public boolean isFinished(double Rx, double Ry) {
        return (((Rx - A_x)*Cx + (Ry - A_y)*Cy)*divider >= 1);
    }
    // returned de target hoek
    public double targetHoek(double Rx, double Ry){
        return AngleA + dAngle*fractionTraveled;
    }
    
    //returned hoeveel de robot moet draaien
    public double driveAngle(double Rx, double Ry, double Rhoek){
        double verschil = 
    }
    
    
    public Point driveVector(double Rx, double Ry) {
        double x = (Cx - afstandLine(Rx,Ry)*Cy*kP)*dividerBig;
        double y = (Cy + afstandLine(Rx,Ry)*Cx*kP)*dividerBig;
        return new Point(x,y);
    }
    
}
