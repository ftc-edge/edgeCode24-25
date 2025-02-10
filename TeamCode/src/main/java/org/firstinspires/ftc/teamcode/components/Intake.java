package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import java.util.*;

@Config
public class Intake {
    public Servo inWrist;
    public Servo inFinger;
    public Servo lArm, rArm;

    private static String initialPos = "default";

    // CLAWS (Open/Close)
    private static float intakeClawOpenPos = 0.0f;
    private static float intakeClawClosePos = 0.2f;
    
    // GROUND POSITIONS: (Wrist, Arm)
    private static float groundWristPos = 0.9f;
    private static float groundLArmPos = 0.0f;
    // private static float groundRArmPos = 0.0f;

    // NEUTRAL POSITIONS: (Wrist, Arm)
    private static float neutralLArmPos = 0.2f;
    // private static float neutralRArmPos = 0.2f;

    // PASSOFF POSITIONS: (Wrist, Arm)
    private static float passoffLArmPos = 0.2f;
    // private static float passoffRArmPos = 0.64f;

    public float inFingPos;
    public float wristServoPos;
    public float outShoulderPos;
    public float lArmServoPos;

    public Intake(HardwareMap hardwareMap){
        rArm = hardwareMap.get(Servo.class, "rArm");
        lArm = hardwareMap.get(Servo.class, "lArm");
        inWrist = hardwareMap.get(Servo.class, "inWrist");
        inFinger = hardwareMap.get(Servo.class, "inFinger");

        switch (initialPos) {
            case "default":
                intakeNeutralPos();
                closeInClaw();
                break;
        
            default:
                break;
        }
    }

    public void boundValues() {
        wristServoPos = max(0, min(1, wristServoPos));
        // rArmservoPos = max(0, min(1, rArmservoPos));
        lArmServoPos = max(0, min(1, lArmServoPos));
        inFingPos = max(0, min(1, inFingPos));
    }

    public void openInClaw() {
        inFingPos = intakeClawOpenPos;
    }

    public void closeInClaw() {
        inFingPos = intakeClawClosePos;
    }

    public void toggleInClaw() {
        if (inFingPos == intakeClawOpenPos) {
            closeInClaw();
        } else {
            openInClaw();
        }
    }

    public void intakeGroundPos() {
        wristServoPos = groundWristPos;
//        rArmservoPos = groundRArmPos;
        lArmServoPos = groundLArmPos;
    }

    public void intakeNeutralPos() {
//        rArmservoPos = neutralRArmPos;
        lArmServoPos = neutralLArmPos;
    }

    public void intakePassoffPos() {
    //    rArmservoPos = passoffRArmPos;
        lArmServoPos = passoffLArmPos;
    }

    public void incrementWrist(float increment) {
        wristServoPos += increment;
    }

    public void incrementLArm(float increment) {
        lArmServoPos += increment;
    }

    // public void incrementRArm(float increment) {
    //     rArmServoPos += increment;
    // }



    public void update(){
        boundValues();
        inWrist.setPosition(wristServoPos);
        inFinger.setPosition(inFingPos);
        lArm.setPosition(lArmServoPos);
        rArm.setPosition(1 - lArmServoPos);
    }
}
