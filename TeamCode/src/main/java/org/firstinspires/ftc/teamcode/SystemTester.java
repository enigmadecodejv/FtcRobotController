package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SystemTester", group = "enigma")
public class SystemTester extends LinearOpMode {

    int numMotors = 8;
    public DcMotor[] motors;
    int numServos = 3;
    public Servo[] servos;
    public boolean isMotorArraySelected = true;
    int arrayPos = 0;
    Gamepad pastGamepad;
    public String lastError = "";

    int numDPadUpChanges = 0;

    public int getSelectedArraySize() {
        return (isMotorArraySelected) ? numMotors : numServos;
    }
    public void changeArrayPosition(boolean increment) {
        int arraySize = getSelectedArraySize();
        if (increment) {
            arrayPos = (arrayPos < (arraySize-1)) ? (arrayPos+1) : (arraySize-1);
        } else {
            arrayPos = (arrayPos != 0) ? (arrayPos - 1) : 0;
        }
    }
    public void runOpMode(){
        pastGamepad = new Gamepad();
        motors = new DcMotor[numMotors];
        motors[0] = hardwareMap.get(DcMotor.class, "FrontRight");//needs to be reversed
        motors[1] = hardwareMap.get(DcMotor.class, "RearRight"); //rearight needs to be reversed
        motors[2] = hardwareMap.get(DcMotor.class, "FrontLeft");
        motors[3] = hardwareMap.get(DcMotor.class,"RearLeft");
        motors[4] = hardwareMap.get(DcMotor.class, "OuttakeLeft"); //good for direction
        motors[5] = hardwareMap.get(DcMotor.class, "OuttakeRight");
        motors[6] = hardwareMap.get(DcMotor.class, "IntakeLeft");//reverse
        motors[7] = hardwareMap.get(DcMotor.class, "IntakeRight");
        servos = new Servo[numServos];
        servos[0] = hardwareMap.get(Servo.class, "OuttakeGateRight");
        servos[1] = hardwareMap.get(Servo.class, "OuttakeGateLeft");
        servos[2] = hardwareMap.get(Servo.class, "RGBLightIndicator");

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addLine("Controls:");
            telemetry.addLine("\tx => selects motors");
            telemetry.addLine("\ty => selects servos");
            telemetry.addLine("\tdpad up/down => change index");
            telemetry.addLine("\tleft joystick up/down => change power/position");
            telemetry.addLine("=================================");

            if (gamepad1.x) {
                isMotorArraySelected = true;
                arrayPos = 0;
            } else if (gamepad1.y) {
                isMotorArraySelected = false;
                arrayPos = 0;
            }

            if (gamepad1.dpad_up && !pastGamepad.dpad_up) {
                changeArrayPosition(true);
                ++numDPadUpChanges;
            }
            if (gamepad1.dpad_down && !pastGamepad.dpad_down) {
                changeArrayPosition(false);
            }

            if (isMotorArraySelected) {
                motors[arrayPos].setPower(gamepad1.left_stick_y);
                telemetry.addLine()
                        .addData("Motor", arrayPos)
                        .addData("Power", motors[arrayPos].getPower());
            } else {
                servos[arrayPos].setPosition((gamepad1.left_stick_y+1.0)/2.0);
                telemetry.addLine()
                        .addData("Servo", arrayPos)
                        .addData("Position", servos[arrayPos].getPosition());
            }

            try {
                pastGamepad.copy(gamepad1);
            }catch(Exception e){
                lastError = e.getMessage();
            }

            telemetry.addLine(lastError);
            telemetry.update();
        }
    }
}
