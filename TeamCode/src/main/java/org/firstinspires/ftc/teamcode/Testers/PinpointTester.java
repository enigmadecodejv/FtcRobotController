package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp (group = "enigma", name = "PinpointTester")
public class PinpointTester extends LinearOpMode {
    public GoBildaPinpointDriver pinpoint;
    public void runOpMode(){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "PinPoint");
        pinpoint.resetPosAndIMU();
        pinpoint.setOffsets(6.75, 2.9375, DistanceUnit.INCH);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Pinpoint position",pinpoint.getPosition());
            pinpoint.update();
            telemetry.update();
        }
    }
}
