package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class decodeIntake {
    public Gamepad gamepad2;
    public DcMotor intake;
    public void abstractTheft (com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, Gamepad gamepad2) {
        this.gamepad2 = gamepad2;
    }
    public void intake (com.qualcomm.robotcore.hardware.HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "intake");
        if (gamepad2.right_trigger > 0.5){
            intake.setPower(1);
        }
    }
}
