package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import java.util.*;
@TeleOp
public class slideEncoderTest extends OpMode{

    private DcMotorEx slideMotor;
    private DcMotorEx slideMotor2;

    private int slideMotorPos;

    public void init() {
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode((DcMotorEx.RunMode.STOP_AND_RESET_ENCODER));
        slideMotor2 = hardwareMap.get(DcMotorEx.class, "underSlide");
        slideMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setDirection(DcMotorEx.Direction.REVERSE);
//        slideMotor.setTargetPosition(537);
//        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        slideMotor.setPower(0.1);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void loop(){
        slideMotorPos = slideMotor.getCurrentPosition();
        telemetry.addData("slide position", slideMotorPos);
        telemetry.addData("Under Slide: ", slideMotor2.getCurrentPosition());
        telemetry.addData("time", System.nanoTime());
        telemetry.update();
    }
}
