package org.firstinspires.ftc.teamcode.opModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.tests.huskyLensTest;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import java.util.*;

//TODO: allow for manual adjustments for most things (e.g. claw on round 5)

@TeleOp()
public class standardDrive extends OpMode {
    private DcMotor FRmotor;
    private DcMotor FLmotor;
    private DcMotor BRmotor;
    private DcMotor BLmotor;
    private DcMotor slideMotor;
    private DcMotor underSlide;
    private DcMotor intakeMotor;

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
    float elbPosition = 0.2f;
    float wriPosition = 0f;
    float fing1Position = 0.43f;
    float fing2Position = 0.95f;
    float outShoPosition = 0.5f;
    float outWriPosition = 0f;
    float outFing1Position = 0.2f;
    float outFing2Position = 0.4f;

    float extensionLimit = 0f;

    float slideMotorPosition;
    float initSlideMotorPosition;
    float horSlideMotorPosition;
    float initHorSlideMotorPosition;
    float slideMotorPower = 0;

    float movementLimiter = 0.7f;
    float vertSlideLimiter = 0.5f;

    long passoffStartTime;
    // long CRmotorStartTime = getTime() + 3000000;
    // double CRmotorRotationTime;

    HashMap<String, Boolean> buttonPressed = new HashMap<String, Boolean>();

    boolean wristREADY = false;
    boolean pressedY = false; // for only getting one event per press
    boolean pressedA = false;
    boolean pressedRT = false;
    boolean pressedRB = false;
    boolean pressedLB = false;
    boolean isPassing = false;
    
    boolean hangMode = false;

    double timeElapsed = -1;
    // HuskyLens.Block[] blocks;

    // public void blockinitize() {
    //      blocks = huskyLens.blocks();
    //  }

    private boolean equals(float a, float b) {
        return (a - b <= 0.01);
    }

    private boolean lessorequals(float a, float b) {
        return ( equals(a,b) || a < b );
    }
    public boolean pressed(String button){
        boolean gamepadState;
        switch (button) {
            case "1y":
                gamepadState = gamepad1.y;
                break;

            case "1x":
                gamepadState = gamepad1.x;
                break;

            case "1a":
                gamepadState = gamepad1.a;
                break;

            case "1b":
                gamepadState = gamepad1.b;
                break;

            case "2a":
                gamepadState = gamepad2.a;
                break;

            case "2b":
                gamepadState = gamepad2.b;
                break;

            case "2x":
                gamepadState = gamepad2.x;
                break;

            case "2y":
                gamepadState = gamepad2.y;
                break;

            case "2rb":
                gamepadState = gamepad2.right_bumper;
                break;

            default:
                gamepadState = false;
                break;
        }

        if (!buttonPressed.containsKey(button)) {
            buttonPressed.put(button, false);
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
        FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
        FLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");
        BRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BLmotor = hardwareMap.get(DcMotor.class, "BLmotor");
        BLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLmotor.setDirection(DcMotor.Direction.REVERSE);
        BLmotor.setDirection(DcMotor.Direction.REVERSE);

        // initSlideMotorPosition = slideMotor.getCurrentPosition();
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        underSlide = hardwareMap.get(DcMotor.class, "underSlide");
        underSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        underSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // initHorSlideMotorPosition = intakeMotor.getCurrentPosition();

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

    private long getTime(){
        return System.nanoTime()/1000000;
    }

    private void openIntakeClaw(){
        fing1Position = 0.8f;
        fing2Position = 0.4f;
    }

    private void closeIntakeClaw(){
        fing1Position = 0.43f;
        fing2Position = 0.95f;
    }

    private void toggleIntakeClaw(){
        if (fing1Position != 0.8f) {
            openIntakeClaw();
        } else {
            closeIntakeClaw();
        }
    }

    private void closeOuttakeClaw(){
        //ACTUALLY OPEN
        outFing1Position = 0.0f;
        outFing2Position = 0.6f;
    }

    private void openOuttakeClaw(){
        // ACTUALLY CLOSE
        outFing1Position = 0.3f;
        outFing2Position = 0.3f;
    }

    private void toggleOuttakeClaw(){
        if (outFing1Position == 0.0f) {
            openOuttakeClaw();
        } else {
            closeOuttakeClaw();
        }
    }

    public void loop() {
        servoControl();
        autoIntake();
        if(gamepad2.a){
            passoff();
        }else if(gamepad2.y){
            place();
        }
        else if(gamepad2.x){
            pickup();
        }
        // if(gamepad2.b){
        //     toggleOuttakeClaw();
        // }

        // if(gamepad1.a) {
        //     shoulderSet(blockDecide(), clawBlock());
        // }
        // if(gamepad1.b) {
        //     wristReady();
        // }
        
        doMove();
        telemetry();
    }

    public void doMove() {

        // if(gamepad1.dpad_left) {
        //     addAngle(1);
        // }


        forward = /* movementLimiter * */ gamepad1.left_stick_y;
        horizontal = /* movementLimiter * */ gamepad1.left_stick_x;
        pivot =/* movementLimiter * */ gamepad1.right_stick_x;

        FRmotor.setPower((forward - horizontal + pivot));
        BRmotor.setPower((forward + horizontal + pivot));
        FLmotor.setPower((forward + horizontal - pivot));
        BLmotor.setPower((forward - horizontal - pivot));

        slideMotorPower = vertSlideLimiter * (gamepad2.left_trigger - gamepad2.right_trigger);

        doVertSlides(slideMotorPower);
        // horSlideMotorPosition = intakeMotor.getCurrentPosition();
        telemetry.addData("intakeMotorPosition", -horSlideMotorPosition);
        telemetry.addData("initial", -initHorSlideMotorPosition);
        // telemetry.addData("true or false", lessorequals((float) -horSlideMotorPosition, (float) -initHorSlideMotorPosition));
        doHorSlides(gamepad2.left_stick_y);
    }


    private void telemetry(){
        telemetry.addData("time", getTime());
        telemetry.addData("CRmotor", timeElapsed);
        telemetry.addData("Elbow Position", elbPosition);
        telemetry.addData("shoulder", shoPosition);
        telemetry.addData("Wrist", wriPosition);
        telemetry.addData("Outtake Shoulder", outShoPosition);
        telemetry.addData("Finger", outFing1Position);
        telemetry.addData("Outtake Wrist", outWriPosition);
        telemetry.addData("SlideMotor", slideMotor.getTargetPosition());
        telemetry.addData("slidemotorpower", slideMotorPower);
        telemetry.addData("CurrentSlideMotorPosition", slideMotorPosition);
        telemetry.addData("strafe", horizontal);
        telemetry.addData("slidespower", vertSlideLimiter);
        telemetry.update();
    }

    private void passoff(){
        outShoPosition = 0.593f;
        outWriPosition = 0.479f;
        closeOuttakeClaw();
    }
    private void place(){
        // Rotate arm above robot
        outShoPosition = 0.98f;
        outWriPosition = 0.22f;

        // Opens the fingers1 just in case
        fing1Position = 0.8f;
        fing2Position = 0.4f;
    }
    private void neutral(){
        // I've commented the two lines out just to use this function as a "open claw" function

        // outShoPosition = 0.5f;
        // outWriPosition = 0f;

        openOuttakeClaw();
    }
    private void pickup(){
        outShoPosition = 0.257f;
        outWriPosition = 0.972f;
        closeOuttakeClaw();
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

        if(gamepad2.left_bumper && !pressedLB){
            if(vertSlideLimiter != 1f){
                vertSlideLimiter = 1f; 
            }
            else{
                vertSlideLimiter = 0.5f;
            }
            pressedLB = true;
        } else if (!gamepad2.left_bumper) {
            pressedLB = false;
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
            toggleIntakeClaw();
            pressedY = true;
        } else if (!gamepad1.y) {
            pressedY = false;
        }

        if(gamepad2.right_bumper && !pressedRB){
            toggleOuttakeClaw();
            pressedRB = true;
        } else if (!gamepad2.right_bumper) {
            pressedRB = false;
        }

        if(gamepad2.dpad_left){
            outWriPosition += 0.001f;
        }
        if(gamepad2.dpad_right){
            outWriPosition -= 0.001f;
        }
        if(gamepad2.dpad_up){
            hangMode = false;
        }
        if(gamepad2.dpad_down){
            hangMode = true;
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
            if(shoPosition!= 0.434f && elbPosition != 0.69f){
                isPassing = true;
                wriPosition = 0;
                elbPosition = 0.69f;
                shoPosition = 0.434f;
                // fing1Position = 0.4f;
                // fing2Position = 0.98f;
            }else{
                isPassing = true;
                // fing1Position = 0.5f;
                // fing2Position = 0.98f;
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
        if(hangMode == true) {
            target = -1;
            // TODO: Control Intake Claw such that it doesn't hit the ground, BELOW CODE TECHNICALLY SHOULD WORK
            wriPosition = 0;
            elbPosition = 0.69f;
            shoPosition = 0.434f;
            
        }
        underSlide.setPower(target);
        // slideMotor.setPower(target);
    }

    private void doHorSlides(float target){
        if(extensionLimit < 100){
            extensionLimit += target;
            intakeMotor.setPower(target);
        }
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
}
