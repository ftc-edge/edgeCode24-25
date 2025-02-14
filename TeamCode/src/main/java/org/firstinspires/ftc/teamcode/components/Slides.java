package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Slides {
    private DcMotorEx vertSlide;
    private DcMotorEx vertSlide2;
    private DcMotorEx horSlide;

    public static int VERTSLIDEMAXPOS = 3310;
    public static int HORSLIDEMAXPOS = 740;
    public static int VERTSLIDEMINPOS = 0;
    public static int HORSLIDEMINPOS = 0;

    public static double initialVertSlidePower = 0.5f;
    public static double initialHorSlidePower = 0.5f;

    public static int specimenVertSlidePos = 2300;
    public static double gotoSpecimenPosPower = 0.6;

    public static int passoffVertSlidePos = 400;

    public static int wallPosVertSlidePos = 700;
    public static int wallPickupVertSlideOffset = 100;

    public static int CVhorSlidePosOffset = 300;
    public static double CVhorSlidePosDivisor = 0.04;

    public static int manualHorSlideSpeed = 10;
    public static int manualVertSlideSpeed = 35;

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
        vertSlide.setPower(initialVertSlidePower);
        vertSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        vertSlide2.setTargetPosition(0);
        vertSlide2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        vertSlide2.setPower(initialVertSlidePower);
        vertSlide2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        horSlide.setTargetPosition(0);
        horSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        horSlide.setPower(initialHorSlidePower);
        horSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        vertSlidePassoffPos();
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

    public void vertSlidePassoffPos(){
        moveVertSlides(passoffVertSlidePos, 0.7);
    }

    public void vertSlideWallPos(){
        moveVertSlides(wallPosVertSlidePos, 1);
    }

    public void vertSlideWallPickup() {
        moveVertSlides(wallPosVertSlidePos + wallPickupVertSlideOffset, 0.7);
    }

    public void horSlidePassoffPos(){
        moveHorSlides(0, 0.9);
    }

    public void setHorSlidesForCV(double yCoordCM){
        int targetPos = max(CVhorSlidePosOffset,horSlide.getCurrentPosition()) + (int) (yCoordCM / CVhorSlidePosDivisor);
        moveHorSlides(targetPos, 0.5);
    }
    public void vertSlidesSpecimenPos(){
        moveVertSlides(specimenVertSlidePos, gotoSpecimenPosPower);
    }

    public void horIncrement(int increment){
        horSlide.setTargetPosition(boundHor(horSlide.getTargetPosition() + increment));
    }

    public int getVertSlidePos(){
        return vertSlide.getTargetPosition();
    }

    public int getHorSlidePosition(){
        return horSlide.getTargetPosition();
    }
}
