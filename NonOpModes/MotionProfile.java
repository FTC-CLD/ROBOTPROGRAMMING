package org.firstinspires.ftc.teamcode.NonOpModes;

// Algorithm used from http://www.dct.tue.nl/New/Lambrechts/Advanced_Setpoints.pdf
public class MotionProfile {
    public double tA, tV, tTotal, xA, s, startPos, v, a;
    private boolean positive;

    public MotionProfile(double posStart, double posEnd, double vMax, double aMax) {
        // Initialises variables
        positive = (posEnd>posStart);
        startPos = posStart;
        v = vMax;
        a = aMax;
        
        // The distance between start and end, but always positive for easier calculations
        s = Math.abs(posStart-posEnd);
        
        // With the formula s = 1/2 a t^2, the time the robot accelerates can be calculated

        tA = Math.sqrt(s/aMax);
        // If the maximum 
        if (tA*aMax > vMax) {
            tA = vMax/aMax;
        }
        xA = aMax*tA*tA;
        tV = (s-xA)/vMax;
        tTotal = tV+tA*2;
        
    }
    
    public double getA(double t) {
        if (t < tA) {
            return positive?a:-a;
        }
        else if (t < tA + tV) {
            return 0;
        }
        else if (t< tTotal){
            return positive?-a:a;
        }
        else return 0;
    }
    
    public double getV(double t) {
        double temp;
        if (t < tA) {
            temp = a*t;
        }
        else if (t < tA + tV) {
            temp = v;
        }
        else if (t< tTotal){
            temp = -a*(t-tTotal);
        }
        else {
            temp = 0;
        }
        // Corrects the velocity for negative and positive velocities
        return positive? temp: -temp;
    }
    
    public double getPos(double t) {
        double temp;
        if (t < tA) {
            temp = a*t*t/2;
        }
        else if (t < tA + tV) {
            temp = xA/2+v*(t-tA);
        }
        else if (t< tTotal) {
            temp = s - a*(t-tTotal)*(t-tTotal)/2;
        }
        else {
            temp = s;
        }
        // Corrects the velocity for negative and positive velocities
        return positive ? temp+startPos: -temp+startPos;
    }
    // todo: write your code here
    
}
