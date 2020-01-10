package org.firstinspires.ftc.teamcode.NonOpModes;

import java.lang.Math;

public class Point {
    public double x, y;
    public Point(double X, double Y) {
        x=X;
        y=Y;
    }
    public Point sub(Point b) {
        return new Point(b.x-x,b.y-y);
    }
    public double magnitude() {
        return Math.sqrt(x*x+y*y);
    }
}