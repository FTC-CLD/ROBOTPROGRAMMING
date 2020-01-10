package org.firstinspires.ftc.teamcode;
import java.lang.Math;
import org.firstinspires.ftc.teamcode.NonOpModes.Point;
public class LeonsStukjes {

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
    
    // niksdoende "pid"
    public double pid(){
        return 1.0;
    }
    
    // stukje om te rijden
    public void richtingrijden(double Ax, double Ay, double Bx, double By, double Rx, double Ry){
        // dit stuk hoef je maar 1 keer te doen
        double Cx = Bx-Ax;
        double Cy = By-Ay;
        // doe alvast 1/divider, zodat je niet nog een keer hoeft te delen
        double dividerBig = 1/Math.sqrt((Cx)*(Cx)+(Cy)*(Cy));
        double divider = ((Cx)*(Cx)+(Cy)*(Cy));
        ///////
        // elke frame:
        double Gx = Rx - Ax;
        double Gy = Ry - Ay;
        
        double distanceToLine = (Gy*Cx - Gx*Cy)*dividerBig;
        
        //check om te kijken of je aangekomen bent:
        if (Gx*Cx + Gy*Cy >= divider){
            //end Process
            
        }
        
    }
    
    

}