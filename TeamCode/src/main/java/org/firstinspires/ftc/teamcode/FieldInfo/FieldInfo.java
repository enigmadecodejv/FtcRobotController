package org.firstinspires.ftc.teamcode.FieldInfo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class FieldInfo {
    /**
     * Utility class containing an April tag's ID, field-centric 3D pose, ball pattern,
     * and whether or not it can be used for navigation.
     */
    public class AprilTag {
        int id = -1;
        Pose3D pose = new Pose3D(
                new Position(DistanceUnit.MM, Double.NaN, Double.NaN, Double.NaN, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, Double.NaN, Double.NaN, Double.NaN, 0)
        );
        boolean useForNavigation = false;
        String pattern = "";

        public AprilTag(int idIn, Pose3D poseIn, boolean useForNavigationIn, String patternIn) {
            id = idIn;
            pose = poseIn;
            useForNavigation = useForNavigationIn;
            pattern = patternIn;
        }
    }
}


/*
    // Pose for AprilTag ID 20
    public static final Pose3D BLUE_GOAL = new Pose3D(
            new Position(DistanceUnit.INCH, 0, 0, 0, 0), // Replace with actual X, Y, Z coordinates
            new YawPitchRollAngles(DistanceUnit.INCH, 0, 0, 0, 0) // Replace Yaw, Pitch, Roll
    );

    // Pose for AprilTag ID 21
    public static final Pose3D TAG_21 = new Pose3D(
            new Position(DistanceUnit.INCH, 0, 0, 0, 0),
            new YawPitchRollAngles(DistanceUnit.INCH, 0, 0, 0, 0)
    );

    // Pose for AprilTag ID 22
    public static final Pose3D TAG_22 = new Pose3D(
            new Position(DistanceUnit.INCH, 0, 0, 0, 0),
            new YawPitchRollAngles(DistanceUnit.INCH, 0, 0, 0, 0)
    );

    // Pose for AprilTag ID 24
    public static final Pose3D RED_GOAL = new Pose3D(
            new Position(DistanceUnit.INCH, 0, 0, 0, 0),
            new YawPitchRollAngles(DistanceUnit.INCH, 0, 0, 0, 0)
    );

    // Pose for AprilTag ID 33
    public static final Pose3D TAG_33 = new Pose3D(
            new Position(DistanceUnit.INCH, 0, 0, 0, 0),
            new YawPitchRollAngles(DistanceUnit.INCH, 0, 0, 0, 0)
    );


    public static Pose3D getPoseById(int id) {
        switch (id) {
            case 20: return TAG_20;
            case 21: return TAG_21;
            case 22: return TAG_22;
            case 24: return TAG_24;
            case 33: return TAG_33;
            default: return null; // Tag ID not mapped
        }
    }
}
*/
