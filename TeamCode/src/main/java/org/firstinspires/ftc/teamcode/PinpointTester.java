package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (group = "enigma", name = "PinpointTester")
public class PinpointTester extends LinearOpMode {
    public GoBildaPinpointDriver pinpoint;
    public void runOpMode(){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "PinPoint");
        pinpoint.resetPosAndIMU();
        pinpoint.setOffsets(-6.41176470588, -2.625, DistanceUnit.INCH);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Pinpoint position",pinpoint.getPosition());
            pinpoint.update();
            telemetry.update();
        }
    }
}
