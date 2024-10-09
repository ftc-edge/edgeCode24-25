package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
public class standardDrive extends OpMode{
    public DcMotor FRmotor;
    public DcMotor FLmotor;
    public DcMotor BRmotor;
    public DcMotor BLmotor;
    @Override
    public void init(){
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");
        BRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BLmotor = hardwareMap.get(DcMotor.class, "BLmotor");
        BLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop(){
        doMove();

        if(gamepad1.right_stick_x > 0f){
            System.out.print("AHHHHHHH");
        }
    }

    public void doMove(){
        float forward;
        float horizontal;
        float pivot;

        forward = -gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;

        FRmotor.setPower(pivot + (-forward + horizontal));
        BRmotor.setPower(pivot + (-forward - horizontal));
        FLmotor.setPower(pivot + (-forward - horizontal));
        BLmotor.setPower(pivot + (-forward + horizontal));
    }

    //public void doSlides(){
        
    //}
}
