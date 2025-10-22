package org.firstinspires.ftc.teamcode;

public class artifactsPos {
    public double x;
    public double y;
    purpleOrGreen PurpleOrGreen;
    public artifactsPos(double x, double y, String purpleorGreen) {
        this.x = x;
        this.y = y;
        if (purpleorGreen.equals("blue")) {
            this.PurpleOrGreen = purpleOrGreen.blue;
        }else if (purpleorGreen.equals("green")){
            this.PurpleOrGreen = purpleOrGreen.green;
        }
    }
    public double radius = 5;
}
