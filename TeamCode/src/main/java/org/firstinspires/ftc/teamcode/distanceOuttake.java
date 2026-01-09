package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class distanceOuttake {

    public double getVelocityShot(double [] robotPos, double [] targetPos){
        double gravity = -368.08;
        double goalDistanceX = targetPos[0] - robotPos[0];
        double goalDistanceY = targetPos[1] - robotPos[0];
        double angle = Math.PI/4;
        return Math.sqrt(Math.pow(goalDistanceX,2) * gravity/(goalDistanceX * Math.sin(2 * angle) - 2 * goalDistanceY * Math.pow(Math.cos(angle),2)));
    }
    public double autoOuttake(double speed){
        double motorRotSpeed = 6000 * 2 * Math.PI/60;
        double motorRadius = 4.25/2.54;
        double power = speed/ (motorRotSpeed * motorRadius);
        if (power > 0.8) {
            power = 0.8;
        } else if (power < -0.8) {
            power = -0.8;
        }
        return power;
    }
}
