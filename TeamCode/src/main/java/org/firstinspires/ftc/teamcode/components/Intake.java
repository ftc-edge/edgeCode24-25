package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    public Servo inWrist;
    public Servo inFinger;
    public Servo lArm, rArm;

    public static String initialPos = "default";

    // CLAWS (Open/Close)
    public static float intakeClawOpenPos = 0.0f;
    public static float intakeClawClosePos = 0.2f;

    public static float intakeClawGrabPos = 1;
    
    public static float intakeClawLightClosePos = 0.14f;

    // GROUND POSITIONS: (Wrist, Arm)
    public static float groundWristPos = 0.9f;
    public static float groundLArmPos = 0.1f;
    // public static float groundRArmPos = 0.0f;

    // NEUTRAL POSITIONS: (Wrist, Arm)
    public static float neutralLArmPos = 0.2f;
    // public static float neutralRArmPos = 0.2f;

    // PASSOFF POSITIONS: (Wrist, Arm)
    public static float passoffLArmPos = 0.686f;
    // public static float passoffRArmPos = 0.64f;
    public static float passoffWristPos = 0.816f;

    // AFTER PASSOFF TIMER:
    public static float afterPassoffWristPos = 0.604f;

    public static double wristBuffer = 0.25;

    public static double clawHangBuffer = 0.5;


    // CV Tuning
    public static double CVwristPosOffset = 90;
    public static double CVwristPosDivisor = 160.5;

    // Cycle Inwrist Positions
    private final float[] inWristPositions = {0.16f, 0.34f, 0.52f, 0.7f, 0.88f, 1.0f};
    private int currentInWristIndex = 0;

    public float inFingPos;
    public float wristServoPos;
    public float outShoulderPos;
    public float lArmServoPos;

    public boolean clawState = false; // open is true, close is false

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
        wristServoPos = max(0.16f, min(1, wristServoPos));
        // rArmservoPos = max(0, min(1, rArmservoPos));
        lArmServoPos = max(0, min(1, lArmServoPos));
        inFingPos = max(0, min(1, inFingPos));
    }

    private void cycleInWristPosition() {
        currentInWristIndex = (currentInWristIndex + 1) % inWristPositions.length;
        wristServoPos = inWristPositions[currentInWristIndex];
    }
    
    public void openInClaw() {
        inFingPos = intakeClawOpenPos;
    }

    public void closeInClaw() {
        inFingPos = intakeClawGrabPos;
    }

    public void hangBlock(){inFingPos = intakeClawClosePos;}

    public void closeInClawLight(){
        inFingPos = intakeClawLightClosePos;
    }

    public boolean toggleInClaw() {
        if (inFingPos == intakeClawOpenPos) {
            closeInClaw();
            return true;
        }
        openInClaw();
        return false;
    }

    public void intakeGroundPos() {
//        wristServoPos = groundWristPos;
//        rArmservoPos = groundRArmPos;
        lArmServoPos = groundLArmPos;
    }

    public void intakeNeutralPos() {
//        rArmservoPos = neutralRArmPos;
        lArmServoPos = neutralLArmPos;
    }

    public void intakePassoffPos() {
    //    rArmservoPos = passoffRArmPos;
        wristServoPos = passoffWristPos;
        lArmServoPos = passoffLArmPos;
    }


    public void intakeAfterPassoffPos(){
        wristServoPos = afterPassoffWristPos;
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

    public void setInWrist(float pos){
        wristServoPos = pos;
    }

    public void setInWristForCV(double angle){
        angle += CVwristPosOffset;
        if (angle >= 180) {
            angle -= 180;
        }
        float pos = (float) (angle / CVwristPosDivisor);
        wristServoPos = pos;
    }


    public void update(){
        boundValues();
        inWrist.setPosition(wristServoPos);
        inFinger.setPosition(inFingPos);
        lArm.setPosition(lArmServoPos);
        rArm.setPosition(1 - lArmServoPos);
    }
}
