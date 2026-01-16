package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class decodeIntake {
    public Gamepad gamepad1;
    public DcMotor intake;
    public HardwareMap hardwareMap;
    public double intakePower;
    public boolean movingForward;
    public decodeIntake (HardwareMap hardwareMap, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        intake = hardwareMap.get(DcMotor.class, "Intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
    }
    public void intake (){
        if (movingForward){
            intakePower = 1;
        }else if (!movingForward){
            intakePower = 0.7;
        }
        if (gamepad1.right_bumper){
            intake.setPower(intakePower);
        }else if (gamepad1.right_trigger > 0.25){
            intake.setPower((-1) * intakePower);
        }else {
            intake.setPower(0);
        }
    }
}
