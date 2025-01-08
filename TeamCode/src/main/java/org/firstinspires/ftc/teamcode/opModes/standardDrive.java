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
    private DcMotor FRmotor;
    private DcMotor FLmotor;
    private DcMotor BRmotor;
    private DcMotor BLmotor;
    private DcMotor slideMotor;
    private DcMotor underSlide;


    // public HuskyLens huskyLens;
    private huskyLensTest hl;

    // public DcMotor backOdo;
    // public DcMotor forwardOdo;

    public Servo clawShoulder;
    public Servo clawElbow;
    public Servo clawWrist;
    public Servo clawFinger1;
    public Servo clawFinger2;
    public Servo outShoulder;
    public Servo outWrist;
    public Servo outFinger1;
    public Servo outFinger2;

    float forward = 0;
    float horizontal = 0;
    float pivot = 0;
    float shoPosition = 0.5f;
    float elbPosition = 0;
    float wriPosition = 0;
    float fing1Position = 0.5f;
    float fing2Position = 0;
    float outShoPosition = 0.5f;
    float outWriPosition = 0f;
    float outFing1Position = 0.15f;
    float outFing2Position = 0.45f;

    float slideMotorPower = 0;

    long passoffStartTime;
    // long CRmotorStartTime = getTime() + 3000000;
    // double CRmotorRotationTime;

    boolean wristREADY = false;
    boolean pressedY = false; // for only getting one event per press
    boolean pressedA = false;
    boolean pressedRT = false;
    boolean isPassing = false;

    double timeElapsed = -1;
    // HuskyLens.Block[] blocks;

    // public void blockInitialize() {
    //      blocks = huskyLens.blocks();
    //  }

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

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        underSlide = hardwareMap.get(DcMotor.class, "underSlide");
        underSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        underSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

        outShoulder = hardwareMap.get(Servo.class, "outShoulder");
        outWrist = hardwareMap.get(Servo.class, "outWrist");
        outFinger1 = hardwareMap.get(Servo.class, "outFinger1");
        outFinger2 = hardwareMap.get(Servo.class, "outFinger2");


    }

    public void loop() {
        servoControl();
        autoIntake();
        if(gamepad2.a){
            passoff();
        }else if(gamepad2.y){
            place();
        }
        else if(gamepad2.b){
            neutral();
        }
        doMove();
        // if(gamepad1.a) {
        //     shoulderSet(blockDecide(), clawBlock());
        // }
        // if(gamepad1.b) {
        //     wristReady();
        // }
        telemetry();
    }

    private long getTime(){
        return System.nanoTime()/1000000;
    }

    public void doMove() {

        // if(gamepad1.dpad_left) {
        //     addAngle(1);
        // }

        forward = gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;

        FRmotor.setPower(pivot + (forward - horizontal));
        BRmotor.setPower(pivot + (forward + horizontal));
        FLmotor.setPower(pivot + (-forward - horizontal));
        BLmotor.setPower(pivot + (-forward + horizontal));

        slideMotorPower = gamepad2.left_trigger - gamepad2.right_trigger;

        doVertSlides(slideMotorPower);

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


    private void telemetry(){

        telemetry.addData("time", getTime());
        telemetry.addData("CRmotor", timeElapsed);
        telemetry.addData("Elbow Position", elbPosition);
        telemetry.addData("shoulder", shoPosition);
        telemetry.addData("Pressed A", pressedA);
        telemetry.addData("Pressed B", gamepad1.b);
        telemetry.addData("Wrist", wriPosition);
        telemetry.addData("Outtake Shoulder", outShoPosition);
        telemetry.addData("Finger", outFing1Position);
        telemetry.addData("Outtake Wrist", outWriPosition);
        telemetry.addData("SlideMotor", slideMotor.getTargetPosition());
        telemetry.addData("slidemotorpower", slideMotorPower);
        telemetry.update();
    }

    private void passoff(){
        outShoPosition = 0.616f;
        outWriPosition = 0.465f;
        outFing1Position = 0.0f;
        outFing2Position = 0.6f;
    }
    private void place(){
        outShoPosition = 0.98f;
        outWriPosition = 0.22f;
        fing1Position = 0.8f;
        fing2Position = 0.4f;
    }
    private void neutral(){
        outShoPosition = 0.5f;
        outWriPosition = 0f;
        outFing1Position = 0.15f;
        outFing2Position = 0.45f;
    }

    // private void passoffTimer(){
    //     if(getTime()-passoffStartTime > 3000){
    //         clawWrist.setPosition(0.16);
    //         clawElbow.setPosition(0.00);
    //         return;
    //     }
    // }

    // private void addAngle(float angle){

    //     CRmotorRotationTime = Math.pow(Math.abs(angle)/16, 0.3562) * 1000;
    //     outShoulder.setPower((angle > 0 ? -1 : 1) * 0.25);
    //     CRmotorStartTime = getTime();
    //     stopCRmotorTimer();
    // }

    // private void stopCRmotorTimer(){
    //     if(getTime()-CRmotorStartTime > CRmotorRotationTime && timeElapsed == -1){
    //         timeElapsed = getTime()-CRmotorStartTime;
    //         outShoulder.setPower(0);
    //     }
    // }

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

        if(gamepad1.a && !pressedA){
            if(wriPosition != 0.5f){
                wriPosition = 0.5f;
                elbPosition = 0.2f;
            }
            else{
                wriPosition = 0;
            }
            pressedA = true;
        } else if (!gamepad1.a) {
            pressedA = false;
        }
        if(gamepad1.y && !pressedY){
            if(fing1Position != 0.4f){
                pressedY = true;
                fing1Position = 0.4f;
                fing2Position = 0.98f;
            }else{
                pressedY = true;
                fing1Position = 0.8f;
                fing2Position = 0.4f;
            }

        } else if (!gamepad1.y) {
            pressedY = false;
        }

        if(gamepad2.x && !pressedRT){
            if(outFing1Position != 0.15f){
                pressedRT = true;
                outFing1Position = 0.15f;
                outFing2Position = 0.45f;
            }else{
                pressedRT = true;
                outFing1Position = 0;
                outFing2Position = 0.6f;
                fing1Position = 0.8f;
                fing2Position = 0.4f;
            }
        } else if (!gamepad2.x) {
            pressedRT = false;
        }

        if(gamepad2.dpad_left){
            outWriPosition += 0.001f;
        }else if(gamepad2.dpad_right){
            outWriPosition -= 0.001f;
        }
        if(gamepad2.dpad_up){
            outShoPosition += 0.001f;
        }else if(gamepad2.dpad_down){
            outShoPosition -= 0.001f;
        }

        clawShoulder.setPosition(shoPosition);
        clawElbow.setPosition(elbPosition);
        clawWrist.setPosition(wriPosition);
        clawFinger1.setPosition(fing1Position);
        clawFinger2.setPosition(fing2Position);
        outShoulder.setPosition(outShoPosition);
        outWrist.setPosition(outWriPosition);
        outFinger1.setPosition(outFing1Position);
        outFinger2.setPosition(outFing2Position);
    }

    private void autoIntake(){
        if(gamepad1.x && !isPassing){
            if(shoPosition!= 0.434f && elbPosition != 0.74f){
                isPassing = true;
                wriPosition = 0;
                elbPosition = 0.74f;
                shoPosition = 0.434f;
                fing1Position = 0.4f;
                fing2Position = 0.98f;
            }else{
                isPassing = true;
                fing1Position = 0.5f;
                fing2Position = 0.98f;
                wriPosition = 0.5f;
                elbPosition = 0.2f;
                shoPosition = 0.33f;
            }
        }else if(!gamepad1.x){
            isPassing = false;
        }

        //clawShoulder.setPosition(0.4);
    }

    private void doVertSlides(float target){
        underSlide.setPower(target);
        slideMotor.setPower(target);
    }


}

