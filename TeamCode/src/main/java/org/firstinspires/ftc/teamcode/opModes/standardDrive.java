package org.firstinspires.ftc.teamcode.opModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.tests.huskyLensTest;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

@TeleOp()
public class standardDrive extends OpMode {

    /* MOTORS */

    private DcMotor FRmotor;
    private DcMotor FLmotor;
    private DcMotor BRmotor;
    private DcMotor BLmotor;

    /* SENSORS */

    public HuskyLens huskyLens;
    private huskyLensTest hl;
    public DcMotor backOdo;
    public DcMotor forwardOdo;

    /* CLAW (Servo) */
    
    public Servo clawShoulder;
    public Servo clawElbow;
    public Servo clawWrist;
    public Servo clawFinger1;
    public Servo clawFinger2;

    /* OUTTAKE (Servo) */

    public CRServo outShoulder;
    public Servo outWrist;
    public Servo outFinger1;
    public Servo outFinger2;

    
    float forward = 0;
    float horizontal = 0;
    float pivot = 0;
    float shoPosition = 0.5f;
    float elbPosition = 0;
    float wriPosition = 0;
    float fingPosition = 0;

    long passoffStartTime;
    long CRmotorStartTime = getTime() + 3000000;
    double CRmotorRotationTime;

    boolean wristREADY = false;
    Dictionary<Char, Boolean> buttonPressed = new Hashtable<>();
    boolean isPassing = false;


    double timeElapsed = -1;
    // HuskyLens.Block[] blocks;

    // public void blockInitialize() {
    //      blocks = huskyLens.blocks();
    //  }

    public Boolean pressed(Char button){
        Boolean gamepadState;
        switch (button) {
            case 'y':
                gamepadState = gamepad1.y;
                break;
        
            case 'x':
                gamepadState = gamepad1.x;
                break;
            
            case 'a':
                gamepadState = gamepad1.a;
                break;
            
            case 'b':
                gamepadState = gamepad1.b;
                break;
            
            default:
                break;
        }

        if (gamepadState) {
            if (!buttonPressed.get(button)) {
                buttonPressed.put(button, true);
                return true;
            }
            return false;
        }
        // if gamepad isn't pressed
        buttonPressed.put(button, false);
        return false;
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


        // backOdo = hardwareMap.get(DcMotor.class, "backOdo");

        // forwardOdo = hardwareMap.get(DcMotor.class, "forwardOdo");

        // huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        //hl.runOpMode();
        // huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        clawShoulder = hardwareMap.get(Servo.class, "clawShoulder");
        clawElbow = hardwareMap.get(Servo.class, "clawElbow");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        clawFinger1 = hardwareMap.get(Servo.class, "clawFinger1");
        clawFinger2 = hardwareMap.get(Servo.class, "clawFinger2");

        outShoulder = hardwareMap.get(CRServo.class, "outShoulder");
        outWrist = hardwareMap.get(Servo.class, "outWrist");
        outFinger1 = hardwareMap.get(Servo.class, "outFinger1");
        outFinger2 = hardwareMap.get(Servo.class, "outFinger2");

        buttonPressed.put("A",false);


    }

    public void loop() {
        servoControl();
        autoIntake();
        clawFinger1.setPosition(0.96);
        doMove();
        // if(gamepad1.a) {
        //     shoulderSet(blockDecide(), clawBlock());
        // }
        // if(gamepad1.b) {
        //     wristReady();
        // }
    }

    private long getTime(){
        return System.nanoTime()/1000000;
    }

    public void doMove() {

        // if(gamepad1.y) {
        //     setPower((float) Math.PI/-4);
        // }

        forward = gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;

        FRmotor.setPower(pivot + (forward - horizontal));
        BRmotor.setPower(pivot + (forward + horizontal));
        FLmotor.setPower(pivot + (-forward - horizontal));
        BLmotor.setPower(pivot + (-forward + horizontal));
    }

//requires gavins perspective geometry formula
//     private int blockDecide() {
//         int closeBlock = 0;
//         for (int i = 1; i < blocks.length; i++) {
//             if (blocks[i].id == 0) {
//                 if (Math.sqrt((Math.pow(blocks[i].x, 2) - 25600) + Math.pow(blocks[i].y, 2) - 14400) < Math.sqrt((Math.pow(blocks[closeBlock].x, 2) - 25600) + Math.pow(blocks[closeBlock].y, 2) - 14400)) {
//                     closeBlock = i;
//                 }
//             }
//         }
//         return closeBlock;
//     }


//     private int clawBlock() {
//         int greenBlock = 0;
//         for (int i = 1; i < blocks.length; i++) {
//             if (blocks[i].id == 1) {
//                 greenBlock = i;
//             }
//         }
//         return greenBlock;
//     }
// //arbitrary values
//     private void shoulderSet(int closeBlock, int greenBlock) {
//         clawShoulder.setPosition(0.5);
//         for (int i = 0; i < Math.abs(blocks[greenBlock].x - blocks[closeBlock].x); i += 0.01) {
//             if (blocks[greenBlock].x > blocks[closeBlock].x) {
//                 clawShoulder.setPosition(0.3 - i);
//             }
//             if (blocks[greenBlock].x < blocks[closeBlock].x) {
//                 clawShoulder.setPosition(0.3 + i);
//             }
//             if (blocks[greenBlock].x == blocks[closeBlock].x){
//                 wristSet();
//                 return;
//             }
//         }
//     }
// //these are arbitrary values; will require testing to figure out
//     private void wristSet(){
//         clawFinger1.setPosition(0.2);
//         clawFinger2.setPosition(0.2);
//         clawWrist.setPosition(0);
//         if(wristREADY){
//             clawFinger1.setPosition(0.5);
//             clawFinger2.setPosition(0.5);
//             clawWrist.setPosition(0.5);
//             clawShoulder.setPosition(0.5);
//             wristREADY = false;
//         }
//     }

    // private void wristReady(){
    //     wristREADY = true;
    // }


    private void servoTesting(){
        // if(gamepad1.x){
        //     passoff();
        // }

        // passoffTimer();
        // stopCRmotorTimer();

        telemetry.addData("time", getTime());
        telemetry.addData("CRmotor", timeElapsed);
        telemetry.addData("Calculated Rotation Time", String.valueOf(CRmotorRotationTime));
        telemetry.addData("Elbow Position", elbPosition);
        telemetry.addData("Finger", fingPosition);
        telemetry.addData("shoulder", shoPosition);
        telemetry.addData("Pressed A", pressedA);
        telemetry.addData("Pressed B", gamepad1.b);
        telemetry.addData("Wrist", wriPosition);
        telemetry.update();
    }

    // private void passoff(){
    //     // get Block
    //     // move intake arm
    //     // Elbow from 0.00 to 0.93
    //     // Wrist to 0.99
    //     // move slide arm
    //     clawFinger2.setPosition(0);
    //     clawShoulder.setPosition(0);
    //     clawElbow.setPosition(0.93);
    //     clawWrist.setPosition(0.99);
    //     passoffStartTime = getTime();
    // }

    // private void passoffTimer(){
    //     if(getTime()-passoffStartTime > 3000){
    //         clawWrist.setPosition(0.16);
    //         clawElbow.setPosition(0.00);
    //         return;
    //     }
    // }

    private void setPower(float angle){

        CRmotorRotationTime = Math.pow(Math.abs(angle)/16, 0.3562) * 1000;
        outShoulder.setPower((angle > 0 ? -1 : 1) * 0.25);
        CRmotorStartTime = getTime();
    }

    private void stopCRmotorTimer(){
        if(getTime()-CRmotorStartTime > CRmotorRotationTime && timeElapsed == -1){
            timeElapsed = getTime()-CRmotorStartTime;
            outShoulder.setPower(0);
        }
    }

    private void servoControl(){
        if(gamepad1.right_bumper){
            shoPosition -= 0.005;
        }else if(gamepad1.left_bumper){
            shoPosition += 0.005;
        }
        if(gamepad1.dpad_up){
            elbPosition += 0.01;
        }else if(gamepad1.dpad_down){
            elbPosition -= 0.01;
        }
        if(elbPosition > 1){
            elbPosition = 1;
        }else if(elbPosition < 0){
            elbPosition = 0;
        }

        if(pressed("a")){
            if(wriPosition != 1){
                wriPosition = 1;
            }
            else{
                wriPosition = 0.15f;
            }
        }

        if(pressed("y")){
            if(fingPosition != 1){
                fingPosition = 1;
            }else{
                fingPosition = 0;
            }
        }

        clawShoulder.setPosition(shoPosition);
        clawElbow.setPosition(elbPosition);
        clawWrist.setPosition(wriPosition);
        clawFinger2.setPosition(fingPosition);

    }

    private void autoIntake(){
        if(pressed('x')){
            if(shoPosition!= 0.45f && elbPosition != 0.8f){
                wriPosition = 0.15f;
                elbPosition = 0.8f;
                shoPosition = 0.45f;
            }else{
                fingPosition = 1;
                wriPosition = 0.75f;
                elbPosition = 0.35f;
                shoPosition = 0.33f;
            }
        }

        //clawShoulder.setPosition(0.4);
    }

}

