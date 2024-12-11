package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.tests.huskyLensTest;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class standardDrive extends OpMode {
    public DcMotor FRmotor;
    public DcMotor FLmotor;
    public DcMotor BRmotor;
    public DcMotor BLmotor;
    public HuskyLens huskyLens;
    private huskyLensTest hl;

    public Servo clawShoulder;
    public Servo clawElbow;
    public Servo clawWrist;
    public Servo clawFinger1;
    public Servo clawFinger2;

    //HuskyLens.Block[] blocks;

    //public void blockInitialize() {
//        blocks = huskyLens.blocks();
//    }

    @Override
    public void init() {
        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");
        BRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BLmotor = hardwareMap.get(DcMotor.class, "BLmotor");
        BLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        hl.runOpMode();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        clawShoulder = hardwareMap.get(Servo.class, "clawShoulder");


    }

    public void loop() {
        doMove();
//        if(gamepad1.a) {
//            shoulderSet(blockDecide(), clawBlock());
//        }
//        if(gamepad1.b) {
//            wristSet();
//        }
    }


    public void doMove() {
        float forward;
        float horizontal;
        float pivot;

        forward = gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;

        FRmotor.setPower(pivot + (-forward + horizontal));
        BRmotor.setPower(pivot + (-forward - horizontal));
        FLmotor.setPower(pivot + (-forward - horizontal));
        BLmotor.setPower(pivot + (-forward + horizontal));
    }

    //public void doSlides(){

    //}

//    private int blockDecide() {
//        int closeBlock = 0;
//        for (int i = 1; i < blocks.length; i++) {
//            if (blocks[i].id == 0) {
//                if (Math.sqrt((Math.pow(blocks[i].x, 2) - 25600) + Math.pow(blocks[i].y, 2) - 14400) < Math.sqrt((Math.pow(blocks[closeBlock].x, 2) - 25600) + Math.pow(blocks[closeBlock].y, 2) - 14400)) {
//                    closeBlock = i;
//                }
//            }
//        }
//        return closeBlock;
//    }


//    private int clawBlock() {
//        int greenBlock = 0;
//        for (int i = 1; i < blocks.length; i++) {
//            if (blocks[i].id == 1) {
//                greenBlock = i;
//            }
//        }
//        return greenBlock;
//    }

//    private void shoulderSet(int closeBlock, int greenBlock) {
//        clawShoulder.setPosition(0.5);
//        for (int i = 0; i < Math.abs(blocks[greenBlock].x - blocks[closeBlock].x); i += 0.01) {
//            if (blocks[greenBlock].x > blocks[closeBlock].x) {
//                clawShoulder.setPosition(0.5 - i);
//            }
//            if (blocks[greenBlock].x < blocks[closeBlock].x) {
//                clawShoulder.setPosition(0.5 + i);
//            }
//        }
//    }

//    private void wristSet(){
//        clawFinger1.setPosition(0.75);
//        clawFinger2.setPosition(0.75);
//        clawWrist.setPosition(0); //sikibidi toilet
//        if(gamepad1.b){
//            clawFinger1.setPosition(0.5);
//            clawFinger2.setPosition(0.5);
//            clawWrist.setPosition(0.5);
//            clawShoulder.setPosition(0.5);
//        }
//    }
}

