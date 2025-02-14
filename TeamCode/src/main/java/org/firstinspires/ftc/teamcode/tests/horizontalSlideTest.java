package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class horizontalSlideTest extends OpMode{

    private DcMotorEx horSlide;

    public void init(){
        horSlide = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        telemetry.addData("horslide", horSlide.getCurrentPosition());
        telemetry.update();
    }
}
