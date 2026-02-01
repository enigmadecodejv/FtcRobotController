package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.1)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.05, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0,0.6,0.01))
            .centripetalScaling(0.0005)
            .forwardZeroPowerAcceleration(-39.228)
            .lateralZeroPowerAcceleration(-70.155);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4)
            .strafePodX(1.75)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .rightFrontMotorName("FrontRight")
            .rightRearMotorName("RearRight")
            .leftRearMotorName("RearLeft")
            .leftFrontMotorName("FrontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(47.125)
            .yVelocity(39.301);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.3, 2);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
