package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import org.firstinspires.ftc.teamcode.components.Slides;

import java.util.*;

@Config
public class Outtake {
    Slides slides;
    public Servo outShoulder;
    public Servo outWrist;
    public Servo outFinger1, outFinger2;
    
    public float outFing1Pos;
    public float outFing2Pos;
    public float outShoulderPos;
    public float outWristPos = 0f;

    public static float outShoulderPlacePos = 0.32f;
    public static float outShoulderSpecimenPos = 0.638f;
    public static float outShoulderHookInPos = 0.6f;
    public static float outShoulderWallPos = 1f;

    public static float outWristPassoffPos = 0.33f;
    public static float outWristWallPos = 0.858f;
    public static float outWristPlacePos = 0.34f;
    public static float outWristSpecimenPos = 0.31f;
    public static float outWristHookInPos = 0.31f;

    // Following are Opposite
    public static float outFing1OpenPos = 0.15f;
    public static float outFing2OpenPos = 0.8f;
    public static float outFing1ClosePos = 0.4f;
    public static float outFing2ClosePos = 0.5f;

    public static double clawBuffer = 1;
    public static double wallPosBuffer = 1;

    public Outtake(HardwareMap hardwareMap){
        outShoulder = hardwareMap.get(Servo.class, "outShoulder");
        outWrist = hardwareMap.get(Servo.class, "outWrist");
        outFinger1 = hardwareMap.get(Servo.class, "outFinger1");
        outFinger2 = hardwareMap.get(Servo.class, "outFinger2");

        outtakePassoffPos();
        closeOutClaw();
    }

    private void boundValues() {
        outWristPos = max(0.18f, min(1, outWristPos));
        outShoulderPos = max(0, min(1, outShoulderPos));
        outFing1Pos = max(0, min(1, outFing1Pos));
        outFing2Pos = max(0, min(1, outFing2Pos));
    }

    public void closeOutClaw(){
        outFing1Pos = outFing1OpenPos;
        outFing2Pos = outFing2OpenPos;
    }

    public void openOutClaw(){
        outFing1Pos = outFing1ClosePos;
        outFing2Pos = outFing2ClosePos;
    }

    public void outtakePlacePos() {
        outShoulderPos = outShoulderPlacePos;
        outWristPos = outWristPlacePos;
    }

    public void outtakePassoffPos() {
        outShoulderPos = outShoulderPassoffPos;
        outWristPos = outWristPassoffPos;
    }

    public void outtakeSpecimenPos() {
        outShoulderPos = outShoulderSpecimenPos;
        outWristPos = outWristSpecimenPos;
    }

    public void outtakeHookInPos() {
        outShoulderPos = outShoulderHookInPos;
        outWristPos = outWristHookInPos;
    }

    public void outtakeWallPos(){
        outShoulderPos = outShoulderWallPos;
        outWristPos = outWristWallPos;
        openOutClaw();
    }

    public void update(){
        boundValues();
        outShoulder.setPosition(outShoulderPos);
        outWrist.setPosition(outWristPos);
        outFinger1.setPosition(outFing1Pos);
        outFinger2.setPosition(outFing2Pos);
    }

    public void toggleOutClaw(){
        if (outFing1Pos == outFing1ClosePos) {
            closeOutClaw();
        } else {
            openOutClaw();
        }
    }

    public void incrementWristPos(float increment){
        outWristPos += increment;
    }

    public void incrementOutShoPos(float increment) {
        outShoulderPos += increment;
    }
}
