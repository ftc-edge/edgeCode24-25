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
public class Outtake {
    public Servo outShoulder;
    public Servo outWrist;
    public Servo outFinger1, outFinger2;
    
    public float outFing1Pos;
    public float outFing2Pos;
    public float outShoulderPos;
    public float outWristPos;

    public static float outShoulderPlacePos = 0.33f;
    public static float outShoulderPassoffPos = 0.811f;
    public static float outShoulderSpecimenPos = 0.628f;
    public static float outFing1OpenPos = 0.5f;
    public static float outFing2OpenPos = 0.8f;
    public static float outFing1ClosePos = 0.8f;
    public static float outFing2ClosePos = 0.5f;

    public Outtake(HardwareMap hardwareMap){
        outShoulder = hardwareMap.get(Servo.class, "outShoulder");
        outWrist = hardwareMap.get(Servo.class, "outWrist");
        outFinger1 = hardwareMap.get(Servo.class, "outFinger1");
        outFinger2 = hardwareMap.get(Servo.class, "outFinger2");
    }

    private void boundValues() {
        outWristPos = max(0, min(1, outWristPos));
        outShoulderPos = max(0.30f, min(1, outShoulderPos));
        outFing1Pos = max(0, min(1, outFing1Pos));
        outFing2Pos = max(0, min(1, outFing2Pos));
    }

    private void openOutClaw(){
        outFing1Pos = outFing1OpenPos;
        outFing2Pos = outFing2OpenPos;
    }

    private void closeOutClaw(){
        outFing1Pos = outFing1ClosePos;
        outFing2Pos = outFing2ClosePos;
    }

    public void outtakePlacePos() {
        outShoulderPos = outShoulderPlacePos;
    }

    public void outtakePassoffPos() {
        outShoulderPos = outShoulderPassoffPos;
    }

    public void outtakeSpecimenPos() {
        outShoulderPos = outShoulderSpecimenPos;
    }

    public void update(){
        boundValues();
        outShoulder.setPosition(outShoulderPos);
        outWrist.setPosition(outWristPos);
        outFinger1.setPosition(outFing1Pos);
        outFinger2.setPosition(outFing2Pos);
    }

    public void toggleOutClaw(){
        if (outFing1Pos == outFing1OpenPos) {
            closeOutClaw();
        } else {
            openOutClaw();
        }
    }
}
