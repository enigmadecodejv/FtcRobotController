package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class decodeIntake {
    public Gamepad gamepad1;
    public DcMotor intake;
    public HardwareMap hardwareMap;
    public decodeIntake (HardwareMap hardwareMap, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        intake = hardwareMap.get(DcMotor.class, "Intake");
    }
    public void intake (){
        if (gamepad1.right_bumper){
            intake.setPower(1);
        }else if (gamepad1.right_trigger > 0.25){
            intake.setPower(-1);
        }else {
            intake.setPower(0);
        }
    }
}
