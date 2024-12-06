package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class AxonTest extends OpMode{
    public Servo clawBallJoint;
    @Override
    public void init(){
        clawBallJoint = hardwareMap.get(Servo.class, "clawBallJoint");
    }

    @Override
    public void loop() {
        test();
    }

    public void test(){
        if(gamepad1.a){
            clawBallJoint.setPosition(0.5);
        }
        if(gamepad1.b){
            clawBallJoint.setPosition(0.1);
            clawBallJoint.setPosition(0.2);
        }
        if(gamepad1.x){
            clawBallJoint.setPosition(1);
        }
        if(gamepad1.y){
            clawBallJoint.setPosition(0);
        }
    }


}
