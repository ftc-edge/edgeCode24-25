package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import java.util.*;

@Config
public class Drive {
    private DcMotor FRmotor;
    private DcMotor FLmotor;
    private DcMotor BRmotor;
    private DcMotor BLmotor;

    public static float BASE_POWER = 1f;
    public static float SECONDARY_POWER = 0.25f;
    public String power_state = "base";

    public Drive(HardwareMap hardwareMap){
        FRmotor = hardwareMap.get(DcMotor.class, "rightFront");
        FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLmotor = hardwareMap.get(DcMotor.class, "leftFront");
        FLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BRmotor = hardwareMap.get(DcMotor.class, "rightRear");
        BRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BLmotor = hardwareMap.get(DcMotor.class, "leftRear");
        BLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       FRmotor.setDirection(DcMotorSimple.Direction.REVERSE);
       BLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(float forward, float horizontal, float pivot){
        FRmotor.setPower((forward + horizontal + pivot) * getMultiplier());
        BRmotor.setPower((forward + horizontal - pivot) * getMultiplier());
        FLmotor.setPower((forward - horizontal - pivot) * getMultiplier());
        BLmotor.setPower((forward - horizontal + pivot) * getMultiplier());

        // front left is -> front right
        // back left is -> black left
        // front right is -> front left
        // back right is -> back right

    }

    public double getMultiplier(){
        return Objects.equals(power_state, "base") ? BASE_POWER : SECONDARY_POWER;
    }

    public void switchState(){
        power_state = Objects.equals(power_state, "base") ? "secondary" : "base";
    }

    // public void setLimiter(float power){
    //     this.POWER = power;
    // }
}
