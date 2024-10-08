package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ProgrammingBoard;
@TeleOp()
public class standardDrive extends OpMode{
    private ProgrammingBoard board;
    @Override
    public void init(){

    }

    public void loop(){
        doMove();
    }

    public void doMove(){
        double forward = gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x;
        double pivot = gamepad1.left_stick_x;

        board.FLmotor.setPower(pivot + (-forward + horizontal));
        board.FRmotor.setPower(pivot + (-forward - horizontal));
        board.BRmotor.setPower(pivot + (-forward - horizontal));
        board.BLmotor.setPower(pivot + (-forward + horizontal));
    }

    public void doSlides(){
        
    }
}
