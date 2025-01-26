package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.*;
@TeleOp
public class slideEncoderTest extends OpMode{

    private DcMotorEx slideMotor;
    private DcMotorEx slideMotor2;
    private Servo outShoulder;
    private Servo outWrist;

    private ElapsedTime runtime = new ElapsedTime();
    private int slideMotorPos;

    public void init() {
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outShoulder = hardwareMap.get(Servo.class, "outShoulder");
        outWrist = hardwareMap.get(Servo.class, "outWrist");

        outShoulder.setPosition(0.1);
        outWrist.setPosition(0.22);

        runtime.reset();
        while (runtime.seconds() < 0.8) {
            telemetry.addData("Path", "Waiting", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();
        slideMotor.setPower(0.5);
        while ((runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: Elapsed", runtime.seconds());
            telemetry.update();
        }
        slideMotor.setPower(0);

        runtime.reset();
        while (runtime.seconds() < 5) {
            telemetry.addData("Path", "Waiting", runtime.seconds());
            telemetry.update();
        }

        runtime.reset();
        slideMotor.setPower(-0.5);
        while ((runtime.seconds() < 0.9)) {
            telemetry.addData("Path", "Leg 3: Elapsed", runtime.seconds());
            telemetry.update();
        }
        slideMotor.setPower(0);

    }

    public void loop(){
        slideMotorPos = slideMotor.getCurrentPosition();
        telemetry.addData("slide position", slideMotorPos);
        telemetry.addData("Under Slide: ", slideMotor2.getCurrentPosition());
        telemetry.addData("time", System.nanoTime());
        telemetry.update();
    }
}
