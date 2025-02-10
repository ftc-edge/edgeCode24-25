package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import java.util.*;

@Config
public class Slides {
    private DcMotorEx vertSlide;
    private DcMotorEx vertSlide2;
    private DcMotorEx horSlide;

    private static int VERTSLIDEMAXPOS = 2750;
    private static int HORSLIDEMAXPOS = 1000;
    private static int VERTSLIDEMINPOS = 0;
    private static int HORSLIDEMINPOS = 0;

    public Slides(HardwareMap hardwareMap){
        vertSlide = hardwareMap.get(DcMotorEx.class, "slideMotor");
        vertSlide2 = hardwareMap.get(DcMotorEx.class, "underSlide");
        horSlide = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        vertSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vertSlide2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vertSlide.setTargetPosition(0);
        vertSlide.setDirection(DcMotorEx.Direction.REVERSE);
        vertSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        vertSlide.setPower(0.1);
        vertSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        vertSlide2.setTargetPosition(0);
        vertSlide2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        vertSlide2.setPower(0.1);
        vertSlide2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        horSlide.setTargetPosition(0);
        horSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        horSlide.setPower(0.1);
        horSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    private int boundVert(int target){
        return max(min(target, VERTSLIDEMAXPOS), VERTSLIDEMINPOS);
    }

    private int boundHor(int target){
        return max(min(target, HORSLIDEMAXPOS), HORSLIDEMINPOS);
    }

    public void moveVertSlides(int target){
        target = boundVert(target);
        vertSlide.setTargetPosition(target);
        vertSlide2.setTargetPosition(target);
    }

    public void moveVertSlides(int target, double power){
        target = boundVert(target);
        vertSlide.setPower(power);
        vertSlide2.setPower(power);
        vertSlide.setTargetPosition(target);
        vertSlide2.setTargetPosition(target);
    }

    public void vertIncrement(int increment){
        vertSlide.setTargetPosition(boundVert(vertSlide.getTargetPosition() + increment));
        vertSlide2.setTargetPosition(boundVert(vertSlide2.getTargetPosition() + increment));
    }

    public void moveHorSlides(int target, double power){
        target = boundHor(target);
        horSlide.setPower(power);
        horSlide.setTargetPosition(target);
    }

    public void horIncrement(int increment){
        horSlide.setTargetPosition(boundHor(horSlide.getTargetPosition() + increment));
    }
}
