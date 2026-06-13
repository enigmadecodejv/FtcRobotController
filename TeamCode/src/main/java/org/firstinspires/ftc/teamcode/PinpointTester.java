package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (group = "enigma", name = "PinpointTester")
public class PinpointTester extends LinearOpMode {
    public GoBildaPinpointDriver pinpoint;
    public void runOpMode(){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "PinPoint");
        pinpoint.resetPosAndIMU();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Pinpoint position",pinpoint.getPosition());
            pinpoint.update();
        }
    }
}
