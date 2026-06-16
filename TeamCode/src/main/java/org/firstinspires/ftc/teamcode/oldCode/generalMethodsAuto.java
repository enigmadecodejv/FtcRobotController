package org.firstinspires.ftc.teamcode.oldCode;

public class generalMethodsAuto {
    public double RCCY = -4.375;//robot center correction Y
    public double RCCX = 0.42;//robot center correction X
    public double displacementX;
    public double displacementY;
    public double angle;
    public double [] headingChangeRCCXandRCCY(double heading){
        double headingChangeRCCX;
        double headingChangeRCCY;
        if (heading < 0){
            heading += 2*Math.PI;
        }
        if (heading < Math.PI/2 || (heading < 3*Math.PI/2 && heading >= Math.PI)){
            headingChangeRCCX = RCCX * Math.cos(heading) + RCCY * Math.cos(heading);
            headingChangeRCCY = RCCX * Math.sin(heading) + RCCY * Math.sin(heading);
        }else {
            headingChangeRCCX = RCCX * Math.sin(heading) + RCCY * Math.sin(heading);
            headingChangeRCCY = RCCX * Math.cos(heading) + RCCY * Math.cos(heading);
        }
        return new double [] {headingChangeRCCX, headingChangeRCCY};
    }
    public double [] findDisplacementAndAngleAdd(double posX, double posY, double heading, double targetX, double targetY) {
        double displacement;
        double angleAdd;
        //find x and y position compared to robot (x - xnot)
        displacementX = targetX - posX;
        displacementY = targetY - posY;
        //find x and y positon in polar coordinates (r@theta)
        displacement = Math.pow(Math.pow(displacementX, 2) + Math.pow(displacementY, 2), 0.5);
        angle = Math.atan(displacementY / displacementX);
        if (angle > Math.PI * 2) {
            angle -= Math.PI * 2;
        } else if (angle < 0) {
            angle += Math.PI * 2;
        }
        if (displacementX > 0 && displacementY > 0) {
            if (angle >= Math.PI / 2) {
                angle = angle + Math.PI;
            }
        } else if (displacementX < 0 && displacementY > 0) {
            if (angle <= Math.PI / 2 || angle >= Math.PI) {
                angle += Math.PI;
            }
        } else if (displacementX < 0 && displacementY < 0) {
            if (angle <= Math.PI || angle >= 3 * Math.PI / 2) {
                angle -= Math.PI;
            }
        } else if (displacementX >= 0 && displacementY <= 0) {
            if (angle <= 3 * Math.PI / 2) {
                angle -= Math.PI;
            }
        }
        //find a theta based on the robot's rotation and angle
        angleAdd = angle - heading;
        if (angleAdd >= 2 * Math.PI) {
            angleAdd = angleAdd - 2 * Math.PI;
        } else if (angleAdd < 0) {
            angleAdd = angleAdd + 2 * Math.PI;
        }
        return new double [] {displacement, angleAdd};
    }
}
