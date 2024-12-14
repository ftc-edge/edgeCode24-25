package org.firstinspires.ftc.teamcode.opModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.tests.huskyLensTest;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

@TeleOp()
public class standardDrive extends OpMode {
    private DcMotor FRmotor;
    private DcMotor FLmotor;
    private DcMotor BRmotor;
    private DcMotor BLmotor;


    public HuskyLens huskyLens;
    private huskyLensTest hl;

    public DcMotor backOdo;
    public DcMotor forwardOdo;

    public Servo clawShoulder;
    public Servo clawElbow;
    public Servo clawWrist;
    public Servo clawFinger1;
    public Servo clawFinger2;

    float forward = 0;
    float horizontal = 0;
    float pivot = 0;

    boolean wristREADY = false;


    HuskyLens.Block[] blocks;

    public void blockInitialize() {
         blocks = huskyLens.blocks();
     }

    @Override
    public void init() {
        /*

        Forward Facing Odometry | Exp. Hub 0
        Front Sideways Odometry | Cont. Hub 0 | ***OCCUPIED***
        Backward Facing Odometry | Cont. Hub 3

        */

        FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
        FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");
        BRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BLmotor = hardwareMap.get(DcMotor.class, "BLmotor");
        BLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        backOdo = hardwareMap.get(DcMotor.class, "backOdo");

        forwardOdo = hardwareMap.get(DcMotor.class, "forwardOdo");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        hl.runOpMode();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        clawShoulder = hardwareMap.get(Servo.class, "clawShoulder");


    }

    public void loop() {
        doMove();
        if(gamepad1.a) {
            shoulderSet(blockDecide(), clawBlock());
        }
        if(gamepad1.b) {
            wristReady();
        }
    }


    public void doMove() {

        forward = gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;

        FRmotor.setPower(pivot + (forward - horizontal));
        BRmotor.setPower(pivot + (forward + horizontal));
        FLmotor.setPower(pivot + (-forward - horizontal));
        BLmotor.setPower(pivot + (-forward + horizontal));
    }

//requires gavins perspective geometry formula
    private int blockDecide() {
        int closeBlock = 0;
        for (int i = 1; i < blocks.length; i++) {
            if (blocks[i].id == 0) {
                if (Math.sqrt((Math.pow(blocks[i].x, 2) - 25600) + Math.pow(blocks[i].y, 2) - 14400) < Math.sqrt((Math.pow(blocks[closeBlock].x, 2) - 25600) + Math.pow(blocks[closeBlock].y, 2) - 14400)) {
                    closeBlock = i;
                }
            }
        }
        return closeBlock;
    }


    private int clawBlock() {
        int greenBlock = 0;
        for (int i = 1; i < blocks.length; i++) {
            if (blocks[i].id == 1) {
                greenBlock = i;
            }
        }
        return greenBlock;
    }
//arbitrary values
    private void shoulderSet(int closeBlock, int greenBlock) {
        clawShoulder.setPosition(0.5);
        for (int i = 0; i < Math.abs(blocks[greenBlock].x - blocks[closeBlock].x); i += 0.01) {
            if (blocks[greenBlock].x > blocks[closeBlock].x) {
                clawShoulder.setPosition(0.3 - i);
            }
            if (blocks[greenBlock].x < blocks[closeBlock].x) {
                clawShoulder.setPosition(0.3 + i);
            }
            if (blocks[greenBlock].x == blocks[closeBlock].x){
                wristSet();
                return;
            }
        }
    }
//these are arbitrary values; will require testing to figure out
    private void wristSet(){
        clawFinger1.setPosition(0.2);
        clawFinger2.setPosition(0.2);
        clawWrist.setPosition(0);
        if(wristREADY){
            clawFinger1.setPosition(0.5);
            clawFinger2.setPosition(0.5);
            clawWrist.setPosition(0.5);
            clawShoulder.setPosition(0.5);
            wristREADY = false;
        }
    }

    private void wristReady(){
        wristREADY = true;
    }
}

