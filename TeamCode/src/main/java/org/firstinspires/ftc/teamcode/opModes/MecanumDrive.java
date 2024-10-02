package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ProgrammingBoard;
@TeleOp()
public class MecanumDrive extends OpMode{
    private ProgrammingBoard board;
    @Override
    public void init(){

    }

    public void loop(){
        doMove();
    }

    public void doMove(){
        double forward = 0;
        double horizontal = 0;
        double pivot = 0;

        forward = gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;

        board.FLmotor.setPower(pivot + (-forward + horizontal));
        board.FRmotor.setPower(pivot + (-forward - horizontal));
        board.BRmotor.setPower(pivot + (-forward - horizontal));
        board.BLmotor.setPower(pivot + (-forward + horizontal));
    }

    public void doSlides(){
        
    }
}
