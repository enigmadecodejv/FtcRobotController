package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class decodeIntake {
    public Gamepad gamepad2;
    public DcMotor intake;
    public HardwareMap hardwareMap;
    public decodeIntake (HardwareMap hardwareMap, Gamepad gamepad2) {
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        intake = hardwareMap.get(DcMotor.class, "Intake");
    }
    public void intake (){
        if (gamepad2.right_trigger > 0.5){
            intake.setPower(1);
        }else {
            intake.setPower(0);
        }
    }
}
