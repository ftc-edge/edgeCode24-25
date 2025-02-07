package org.firstinspires.ftc.teamcode.tests;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoPositionTest extends OpMode {

    //CON
    public Servo inWrist;
    public Servo inFinger;
    public Servo outShoulder;
    public Servo outWrist;

    //EXP
    public Servo lArm, rArm;
    public Servo outFinger1, outFinger2;

    public float rArmservoPos = 0;
    public float lArmServoPos = 0;

    public float outFing1Pos = 0.0f;
    public float outFing2Pos = 0.0f;

    public float wristServoPos = 0;
    public float outWristPos = 0;


    public void boundValues(){
        wristServoPos = max(0, min(1, wristServoPos));
        rArmservoPos = max(0, min(1, rArmservoPos));
        lArmServoPos = max(0, min(1, lArmServoPos));
        outWristPos = max(0, min(1, outWristPos));
        outFing1Pos = max(0, min(1, outFing1Pos));
        outFing2Pos = max(0, min(1, outFing2Pos));
    }
    // close: 0.8
    // open: 0.5
    public void openOutClaw(){
        outFing1Pos = 0.5f;
        outFing2Pos = 0.8f;
    }

    public void closeOutClaw(){
        outFing1Pos = 0.8f;
        outFing2Pos = 0.5f;
    }

    public void init(){
        outShoulder = hardwareMap.get(Servo.class, "outShoulder");
        outWrist = hardwareMap.get(Servo.class, "outWrist");
        outFinger1 = hardwareMap.get(Servo.class, "outFinger1");
        outFinger2 = hardwareMap.get(Servo.class, "outFinger2");

        rArm = hardwareMap.get(Servo.class, "rArm");
        lArm = hardwareMap.get(Servo.class, "lArm");
        inWrist = hardwareMap.get(Servo.class, "inWrist");
        inFinger = hardwareMap.get(Servo.class, "inFinger");
    }


    public void loop(){
//        outShoulder.setPosition(0);
//        if(gamepad1.a){
//            outWrist.setPosition(0);
//        }
//        if(gamepad1.b){
//            outFinger1.setPosition(0);
//        }
//        if(gamepad1.x){
//            outFinger2.setPosition(0);
//        }
//        if(gamepad1.y){
//            lArm.setPosition(0);
//        }
//        if(gamepad1.dpad_down){
//            rArm.setPosition(0);
//        }
//        if(gamepad1.dpad_up){
//            inWrist.setPosition(0);
//        }
//        if(gamepad1.dpad_left){
//            inFinger.setPosition(0);
//        }
        if(gamepad1.dpad_down){
            wristServoPos -= 0.001f;
        }
        if(gamepad1.dpad_up){
            wristServoPos += 0.001f;
        }

        outFing1Pos += (gamepad1.left_trigger) / 1000;
        outFing2Pos += (gamepad1.right_trigger) / 1000;

        if(gamepad1.dpad_left){
            outWristPos += 0.003f;
        }
        if(gamepad1.dpad_right){
            outWristPos -= 0.003f;
        }

        if(gamepad1.a){
            //floor position
            rArmservoPos = 0.30f;
            lArmServoPos = 0.64f;
        }
        if(gamepad1.b){
            //passoff position
            rArmservoPos = 0;
            lArmServoPos = 0;
        }
        if(gamepad1.x){
            //neutral position
            rArmservoPos = 0.2f;
            lArmServoPos = 0.5f;
        }
        if(gamepad1.y){
            wristServoPos = 0.55f;
        }

        if(gamepad1.left_bumper){
            openOutClaw();
        }

        if(gamepad1.right_bumper){
            closeOutClaw();
        }

        boundValues();

        // TODO: Expand Range of "inWrist" by a lot
        // TODO: Expand Range of "outWrist" (currently 180deg), top of robot heading backwards

        lArm.setPosition(lArmServoPos);
        rArm.setPosition(rArmservoPos);
        inWrist.setPosition(wristServoPos);
        outWrist.setPosition(outWristPos);
        outFinger1.setPosition(outFing1Pos);
        outFinger2.setPosition(outFing2Pos);

        telemetry.addData("inWrist Servo Position", wristServoPos);
        telemetry.addData("outWrist Servo Position", outWristPos);
        telemetry.addData("outFinger1 Servo Position", outFing1Pos);
        telemetry.addData("outFinger2 Servo Position", outFing2Pos);
        telemetry.addData("lArm Servo Position", lArmServoPos);
        telemetry.addData("rArm Servo Position", rArmservoPos);
        telemetry.update();

    }
}

