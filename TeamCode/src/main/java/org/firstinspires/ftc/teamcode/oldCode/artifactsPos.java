package org.firstinspires.ftc.teamcode.oldCode;

import org.firstinspires.ftc.teamcode.oldCode.vision.PurpleOrGreen;

public class artifactsPos {
    public double x;
    public double y;
    PurpleOrGreen purpleOrGreen;
    public artifactsPos(double x, double y, String PurpleorGreen) {
        this.x = x;
        this.y = y;
        if (PurpleorGreen.equals("purple")) {
            this.purpleOrGreen = PurpleOrGreen.purple;
        }else if (PurpleorGreen.equals("green")){
            this.purpleOrGreen = PurpleOrGreen.green;
        }
    }
    public double radius = 5;
}
