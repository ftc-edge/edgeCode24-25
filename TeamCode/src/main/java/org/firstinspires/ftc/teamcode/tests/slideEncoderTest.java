package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class slideEncoderTest extends OpMode {

    private DcMotorEx slideMotor;
    private DcMotorEx slideMotor2;
    private DcMotorEx horSlide;
    private Servo outShoulder;
    private Servo outWrist;

    private ElapsedTime runtime = new ElapsedTime();
    private int slideMotorPos;
    private int slideMotor2Pos;
    private int horSlidePos;

    private int OUTTAKESPECIMENPOS = 1560;
    private int slideMotorTarget = 0;
    private int horSlideTarget = 0;

    public void moveVertSlides(int target){
        slideMotor.setTargetPosition(target);
        slideMotor2.setTargetPosition(target);
    }

    public void moveVertSlides(int target, double power){
        slideMotor.setPower(power);
        slideMotor2.setPower(power);
        if(motorA){
            slideMotor.setTargetPosition(target);
        }
        if(motorB){
            slideMotor2.setTargetPosition(target);
        }
    }

    public void moveHorSlides(int target, double power){
        horSlide.setPower(power);
        if(motorHor){
            horSlide.setTargetPosition(target);
        }
    }

    boolean motorA = true;
    boolean motorB = true;
    boolean motorHor = true;

    public void init() {
        if(gamepad1.dpad_right){
            motorHor = false;
        }
        if(gamepad1.dpad_down){
            motorA = false;
        }
        if(gamepad2.dpad_down){
            motorB = false;
        }

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor2 = hardwareMap.get(DcMotorEx.class, "underSlide");
        horSlide = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(motorA){
            slideMotor.setTargetPosition(0);
            slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(0.1);
            slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        if(motorB){
            slideMotor2.setTargetPosition(0);
            slideMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slideMotor2.setPower(0.1);
            slideMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        if(motorHor){
            horSlide.setTargetPosition(0);
            horSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            horSlide.setPower(0.1);
            horSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        runtime.reset();

    }

    public void loop(){
        slideMotorPos = slideMotor.getCurrentPosition();
        slideMotor2Pos = slideMotor2.getCurrentPosition();
        horSlidePos = horSlide.getCurrentPosition();

        telemetry.addData("slide position", slideMotorPos);
        telemetry.addData("slide2 position", slideMotor2Pos);
        telemetry.addData("horSlide Position", horSlidePos);
        telemetry.addData("slide off/on", motorA);
        telemetry.addData("slide2 off/on", motorB);
        telemetry.addData("horSlide off/on", motorHor);
        telemetry.addData("time", System.nanoTime());
        telemetry.update();

        boolean setTargetByDPAD = true;
        if(setTargetByDPAD){
            slideMotorTarget += gamepad1.dpad_up ? 3 : (gamepad1.dpad_down ? -3 : 0);
            horSlideTarget += gamepad1.dpad_left ? 3 : (gamepad1.dpad_right ? -3 : 0);
            moveVertSlides(slideMotorTarget, 0.5);
            moveHorSlides(horSlideTarget, 0.5);
            return;
        }

        if(gamepad1.a){
            moveHorSlides(0, 0.3);
        }

        if(gamepad1.b){
            moveHorSlides(300, 0.6);
        }

        if(gamepad1.x){
            moveHorSlides(800, 0.5);
        }

        if(gamepad1.y){
            moveHorSlides(300, 0.9);
        }

        if(gamepad1.left_bumper){
            moveVertSlides(1000, 0.3);
        }

        if(gamepad1.right_bumper){
            moveVertSlides(300, 0.9);
        }

        if(gamepad2.left_bumper) {
            moveVertSlides(100, 0.3);
        }
        if(gamepad2.right_bumper) {
            moveVertSlides(700, 0.9);
        }

    }
}
