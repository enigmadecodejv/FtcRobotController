package org.firstinspires.ftc.teamcode.oldCode;

public class Cart2D {
    double x;
    double y;
    public Cart2D(double x, double y){
        this.x = x;
        this.y = y;
    }
    public static Cart2D subtractPose(Cart2D Pose, Cart2D Pose2){
        return new Cart2D(Pose.x - Pose2.x, Pose.y - Pose2.y);
    }
}
