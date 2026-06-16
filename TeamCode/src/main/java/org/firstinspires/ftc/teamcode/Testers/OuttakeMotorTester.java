package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "OuttakeMotorTester", group = "Enigma")
public class OuttakeMotorTester extends LinearOpMode {
    public DcMotor outtakeLeft;
    public DcMotor outtakeRight;
    public void runOpMode(){
        outtakeLeft = hardwareMap.get(DcMotor.class, "OuttakeLeft");
        outtakeRight = hardwareMap.get(DcMotor.class, "OuttakeRight");
        outtakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()){
            outtakeLeft.setPower(0.5);
            outtakeRight.setPower(0.5);
            telemetry.addLine("Inside while");
            telemetry.update();
        }
    }
}
