package org.firstinspires.ftc.teamcode;

public class cart2D {
    double x;
    double y;
    public cart2D(double x, double y){
        this.x = x;
        this.y = y;
    }
    public static cart2D subtractPose(cart2D Pose, cart2D Pose2){
        return new cart2D(Pose.x - Pose2.x, Pose.y - Pose2.y);
    }
}
