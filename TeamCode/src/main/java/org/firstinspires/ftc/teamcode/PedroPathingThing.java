package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

public class PedroPathingThing {
    public Pose Position = new Pose(20,20, 20);
    private Follower thing;
    public PedroPathingThing() {
        thing.followPath(thing.pathBuilder().addPath(new BezierLine(thing.getPose(), Position)).setLinearHeadingInterpolation(thing.getHeading(), Position.minus(thing.getPose()).getAsVector().getTheta()).build());
    }
}