package org.firstinspires.ftc.teamcode.opModes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.tests.huskyLensTest;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
public class OutFingerTester extends OpMode{
    public Servo outFinger1;
    public Servo outFinger2;

    float outFing1Position = 0.0f;
    float outFing2Position = 1.0f;

    @Override
    public void init(){
        outFinger1 = hardwareMap.get(Servo.class, "outFinger1");
        outFinger2 = hardwareMap.get(Servo.class, "outFinger2");
    }

    public void loop(){
        telemetry.addData("pos1", outFing1Position);
        telemetry.addData("pos2", outFing2Position);
        telemetry.update();

        if (gamepad1.dpad_up) {
            outFing1Position += 0.01f;
        }
        if (gamepad1.dpad_down) {
            outFing1Position -= 0.01f;
        }
        if (gamepad1.dpad_left) {
            outFing1Position += 0.01f;
        }
        if (gamepad1.dpad_up) {
            outFing1Position -= 0.01f;
        }

        outFinger1.setPosition(outFing1Position);
        outFinger2.setPosition(outFing2Position);

    }

}
