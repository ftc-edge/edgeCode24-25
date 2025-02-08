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

    public float lArmServoPos = 0;

    public float outFing1Pos = 0.8f;
    public float outFing2Pos = 0.5f;
    public float inFingPos = 0.0f;
    public float wristServoPos = 0;
    public float outShoulderPos = 0.33f;

    public void boundValues() {
        wristServoPos = max(0, min(1, wristServoPos));
//        rArmservoPos = max(0, min(1, rArmservoPos));
        lArmServoPos = max(0, min(1, lArmServoPos));
        outShoulderPos = max(0.30f, min(1, outShoulderPos));
        outFing1Pos = max(0, min(1, outFing1Pos));
        outFing2Pos = max(0, min(1, outFing2Pos));
        inFingPos = max(0, min(1, inFingPos));
    }

    public void openOutClaw() {
        outFing1Pos = 0.5f;
        outFing2Pos = 0.8f;
    }

    public void closeOutClaw() {
        outFing1Pos = 0.8f;
        outFing2Pos = 0.5f;
    }

    public void openInClaw() {
        inFingPos = 0.0f;
    }

    public void closeInClaw() {
        inFingPos = 0.2f;
    }

    public void outtakePlacePos() {
        outShoulderPos = 0.33f;
    }

    public void outtakePassoffPos() {
        outShoulderPos = 0.811f;
    }

    public void outtakeSpecimenPos() {
        outShoulderPos = 0.628f;
    }

    public void intakeGroundPos() {
        wristServoPos = 0.9f;
//        rArmservoPos = 0;
        lArmServoPos = 0.0f;
    }

    public void intakeNeutralPos() {
//        rArmservoPos = 0.2f;
        lArmServoPos = 0.2f;
    }

    public void intakePassoffPos() {
//        rArmservoPos = 0.30f;
        lArmServoPos = 0.64f;
    }

    public void init() {
        outShoulder = hardwareMap.get(Servo.class, "outShoulder");
        outWrist = hardwareMap.get(Servo.class, "outWrist");
        outFinger1 = hardwareMap.get(Servo.class, "outFinger1");
        outFinger2 = hardwareMap.get(Servo.class, "outFinger2");

        rArm = hardwareMap.get(Servo.class, "rArm");
        lArm = hardwareMap.get(Servo.class, "lArm");
        inWrist = hardwareMap.get(Servo.class, "inWrist");
        inFinger = hardwareMap.get(Servo.class, "inFinger");
    }


    public void loop() {
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
        if (gamepad1.dpad_down) {
            lArmServoPos -= 0.001f;
        }
        if (gamepad1.dpad_up) {
            lArmServoPos += 0.001f;
        }

        if (gamepad1.dpad_left) {
            wristServoPos += 0.002f;
        }
        if (gamepad1.dpad_right) {
            wristServoPos -= 0.002f;
        }

        if (gamepad1.a) {
            //floor position
            intakeGroundPos();
        }
        if (gamepad1.b) {
            //just above floor
//            rArmservoPos = 0.27f;
            lArmServoPos = 0.1f;
        }
        if (gamepad1.y) {
            //passoff position
            intakePassoffPos();
            sleep(1000);
            wristServoPos = 0.55f;
        }
        if (gamepad1.x) {
            //neutral position
            intakeNeutralPos();
        }

        if (gamepad2.x) {
            outtakePassoffPos();
        }
        if (gamepad2.y) {
            outtakePlacePos();
        }
        if (gamepad2.a) {
            outtakeSpecimenPos();
        }
        if (gamepad1.left_bumper) {
            openInClaw();
        }

        if (gamepad1.right_bumper) {
            closeInClaw();
        }

        if (gamepad2.left_bumper) {
            openOutClaw();
        }

        if (gamepad2.right_bumper) {
            closeOutClaw();
        }


        boundValues();

        lArm.setPosition(lArmServoPos);
        rArm.setPosition(1 - lArmServoPos);
        inWrist.setPosition(wristServoPos);
        outShoulder.setPosition(outShoulderPos);
        outFinger1.setPosition(outFing1Pos);
        outFinger2.setPosition(outFing2Pos);
        inFinger.setPosition(inFingPos);

        telemetry.addData("inFing Servo Position", inFingPos);
        telemetry.addData("inWrist Servo Position", wristServoPos);
        telemetry.addData("outShoulder Servo Position", outShoulderPos);
        telemetry.addData("outFinger1 Servo Position", outFing1Pos);
        telemetry.addData("outFinger2 Servo Position", outFing2Pos);
        telemetry.addData("lArm Servo Position", lArmServoPos);
//        telemetry.addData("rArm Servo Position", rArmservoPos);
        telemetry.update();

    }

    public void sleep(int milis) {
        try {
            Thread.sleep(100);
        } catch (Exception e) {}
    }
}

