package org.firstinspires.ftc.teamcode.NonOpModes;


public class PointFollower {
    public double radius;
    public Point B;
    public PointFollower(Point Goto, double r) {
        B = Goto;
        radius = r;
    }
    public Point simpleMove(Point R){
        return B.sub(R);
    }
    public boolean isFinished(Point R) {
        return ((R.sub(B)).magnitude() > radius);
    }
}