package org.firstinspires.ftc.teamcode.tests;

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
    public float wristServoPos = 0;


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

        if(gamepad1.a){
            //floor position
            rArmservoPos = 0.35f;
            lArmServoPos = 0.725f;
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
            wristServoPos = 0.55f
        }


        lArm.setPosition(lArmServoPos);
        rArm.setPosition(rArmservoPos);
        inWrist.setPosition(wristServoPos);



        telemetry.addData("Servo Position", wristServoPos);
        telemetry.update();

    }
}

