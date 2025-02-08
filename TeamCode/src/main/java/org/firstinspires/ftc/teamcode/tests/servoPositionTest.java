package org.firstinspires.ftc.teamcode.tests;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoPositionTest extends OpMode {

    public float clamp(float val, float min, float max){
        return max(min, min(max, val));
    }

    public Servo inWrist, inFinger, outShoulder, outWrist, lArm, rArm, outFinger1, outFinger2;

    public float rArmservoPos = 0;
    public float lArmServoPos = 0;

    public float outFing1Pos = 0.8f;
    public float outFing2Pos = 0.5f;
    public float inFingPos = 0.0f;
    public float wristServoPos = 0;
    public float outShoulderPos = 0.811f ;

    public void boundValues(){
        wristServoPos = clamp(wristServoPos, 0, 1);
        rArmservoPos = clamp(rArmservoPos, 0, 1);
        lArmServoPos = clamp(lArmServoPos, 0, 1);
        outShoulderPos = clamp(outShoulderPos, 0.30f, 1);
        outFing1Pos = clamp(outFing1Pos, 0, 1);
        outFing2Pos = clamp(outFing2Pos, 0, 1);
        inFingPos = clamp(inFingPos, 0, 0.2f);
    }

    public void openOutClaw(){
        outFing1Pos = 0.5f;
        outFing2Pos = 0.8f;
    }

    public void closeOutClaw(){
        outFing1Pos = 0.8f;
        outFing2Pos = 0.5f;
    }

    public void toggleOutClaw(){
        if(outFing1Pos == 0.5f){
            closeOutClaw();
        }else{
            openOutClaw();
        }
    }

    public void openInClaw(){
        inFingPos = 0.0f;
    }

    public void closeInClaw(){
        inFingPos = 0.2f;
    }

    public void toggleInClaw(){
        if(inFingPos == 0.0f){
            closeInClaw();
        }else{
            openInClaw();
        }
    }

    public void outtakePlacePos(){
        outShoulderPos = 0.33f;
    }

    public void outtakePassoffPos(){
        outShoulderPos = 0.811f;
    }

    public void outtakeSpecimenPos(){
        outShoulderPos = 0.628f;
    }

    public void intakePassoffPos(){
        wristServoPos = 0.9f;
        rArmservoPos = 0.0f;
        lArmServoPos = 0.0f;
    }

    public void intakeNeutralPos(){
        rArmservoPos = 0.2f;
        lArmServoPos = 0.5f;
    }

    public void intakeGroundPos(){
        rArmservoPos = 0.30f;
        lArmServoPos = 0.64f;
    }

    public void intakeJustAboveGroundPos(){
        rArmservoPos = 0.27f;
        lArmServoPos = 0.58f;
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
        if(gamepad1.dpad_left){
            wristServoPos += 0.002f;
        }
        if(gamepad1.dpad_right){
            wristServoPos -= 0.002f;
        }

        if(gamepad1.b){
            intakeJustAboveGroundPos();
        }
        if(gamepad1.a){
            intakeGroundPos();
        }
        if(gamepad1.y){
            intakePassoffPos();
        }
        if(gamepad1.x){
            intakeNeutralPos();
        }

        if(gamepad1.y){
            wristServoPos = 0.55f;
        }

        if(gamepad2.x){
            outtakePassoffPos();
        }
        if(gamepad2.y){
            outtakePlacePos();
        }
        if(gamepad2.a){
            outtakeSpecimenPos();
        }
        
        if(gamepad1.left_bumper){
            toggleInClaw();
        }

        if(gamepad2.left_bumper){
            toggleOutClaw();
        }

        boundValues();

        lArm.setPosition(lArmServoPos);
        rArm.setPosition(rArmservoPos);
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
        telemetry.addData("rArm Servo Position", rArmservoPos);
        telemetry.update();

    }
}

