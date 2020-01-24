package org.firstinspires.ftc.teamcode.NonOpModes;

public class FollowLine {
    public double Cx, Cy, dividerBig, divider, A_x, A_y, kP, AngleA, AngleB, dAngle, kPa;
    public boolean klaar;
    
    public FollowLine(double Ax, double Ay, double Bx, double By, double myAngleA, double myAngleB, boolean draainaarrechts, double P, double Pa) {
        kP = P;
        kPa = Pa;//dit is voor de hoek
        Cx = Bx-Ax;
        Cy = By-Ay;
        
        //of de robot al ver genoeg is gereden
        klaar = false;
        
        AngleA = myAngleA;
        AngleB = myAngleB;
        dAngle = (AngleB - AngleA + 360)%360;
        if (draainaarrechts){
            dAngle = dAngle - 360;
        }

        // doe alvast 1/divider, zodat je niet nog een keer hoeft te delen
        dividerBig = 1/Math.sqrt((Cx)*(Cx)+(Cy)*(Cy));
        divider = 1/((Cx)*(Cx)+(Cy)*(Cy));
    }
    public double afstandLine(double Rx, double Ry){
        return ((Ry - A_y)*Cx - (Rx - A_x)*Cy)*dividerBig;
    }
    //returned waarde tussen 0 en 1(klaar)
    public double afstandGereden(double Rx, double Ry){
        return ((Rx - A_x)*Cx + (Ry - A_y)*Cy)*divider;
    }
    public boolean isFinished(double Rx, double Ry) {
        return (((Rx - A_x)*Cx + (Ry - A_y)*Cy)*divider >=1);
    }
    public double targetHoek(double afstand){
        return (AngleA + afstand*dAngle)%360;
    }
    
    //rA IS DE hoek
    // return x,y,hoek
    public double[] driveVector(double Rx, double Ry, double Ra) {
        
        klaar = isFinished(Rx, Ry) || klaar;
        
        double x = 0;
        double y = 0;
        if (!klaar){
            // simpele P volger, + constante snelheid vooruit
            x = (Cx - afstandLine(Rx,Ry)*Cy*kP)*dividerBig;
            y = (Cy + afstandLine(Rx,Ry)*Cx*kP)*dividerBig;
        }
        
        
        // zorg dat de robot niet verder draait als hij verder rijdt
        double targetA = targetHoek(Math.min(afstandGereden(Rx, Ry),1));
        // simpele P volger
        double a = ((targetA - Ra + 540)%360 - 180)*kPa;
        
        
        double threshold = 2;
        if (Ra-targetA > threshold && targetA - Ra > threshold && klaar){
            // termineer het rijblok, de robot is aangekomen
        }
        //returned x en y absoluut
        return new double[] {x,y,a};
    }
    
}
