package org.firstinspires.ftc.teamcode.opModes;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.*;

import java.util.Objects;

// TODO: Reverse Pivot
@TeleOp()
public class NewStandardDrive extends OpMode {

    Drive drive;
    Slides slides;
    Intake intake;
    Outtake outtake;

    RedObjectTracking cv;

    private ElapsedTime runtime = new ElapsedTime();
    int passoffStage = 0;
    int wallPosStage = 0;

    boolean onWall = false;

    boolean pressed2square = false;
    boolean pressed1square = false;
    boolean pressed1triangle = false;
    boolean pressed1circle = false;
    boolean pressed1cross = false;
    boolean pressed1lb = false;
    boolean pressed1rb = false;

    boolean pressed2x = false;
    boolean pressed2rb = false;
    boolean pressed2lb = false;

    boolean hangMode = false;

    double yDisplacement = -1;

    @Override
    public void init() {
        gamepad2.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        drive = new Drive(hardwareMap);
        slides = new Slides(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        cv = new RedObjectTracking(hardwareMap);
    }

    private long getTime(){
        return System.nanoTime()/1000000;
    }

    public void loop() {
        servoControl();
        doMove();
        telemetry();
    }

    public void doMove() {
        drive.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        slides.vertIncrement(Slides.manualVertSlideSpeed * (int) (gamepad2.left_trigger - gamepad2.right_trigger));
        slides.horIncrement(Slides.manualHorSlideSpeed * (int) gamepad2.left_stick_y);
        intake.update();
        outtake.update();
        if (hangMode) {
            slides.moveVertSlides(900, 1);
        }
    }

    private void telemetry(){
        telemetry.addData("time", getTime());
        telemetry.addData("vertSlidePos", slides.getVertSlidePos());
        telemetry.addData("horSlidePos", slides.getHorSlidePosition());
        telemetry.addData("outWrist", outtake.outWristPos);
        telemetry.addData("outSho", outtake.outShoulderPos);
        telemetry.addData("inWrist", intake.wristServoPos);
        telemetry.addData("inLArm", intake.lArmServoPos);
        telemetry.addData("detectedAngle", cv.getDetectedAngle());
        telemetry.addData("Detected yDisplacement", yDisplacement);
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.update();
    }

    private void closeInClawWithController(){
        intake.closeInClaw();
        gamepad1.setLedColor(1, 0, 0, 2000);
    }

    private void openInClawWithController(){
        intake.openInClaw();
        gamepad1.setLedColor(0, 1, 0, 2000);
    }

    private void toggleInClawWithController(){
        if (intake.toggleInClaw()){
            gamepad1.setLedColor(1, 0, 0, 2000);
            return;
        }
        gamepad1.setLedColor(0, 1, 0, 2000);
    }

    private void servoControl(){
        if (gamepad1.dpad_down) {
            intake.incrementLArm(-0.001f);
        }
        if (gamepad1.dpad_up) {
            intake.incrementLArm(0.001f);
        }

        if (gamepad1.dpad_left) {
            intake.incrementWrist(0.002f);
        }
        if (gamepad1.dpad_right) {
            intake.incrementWrist(-0.002f);
        }

        if(gamepad2.dpad_up){
            outtake.incrementWristPos(0.002f);
        }
        if(gamepad2.dpad_down){
            outtake.incrementWristPos(-0.002f);
        }

        if(gamepad2.dpad_right){
            outtake.incrementOutShoPos(0.002f);
        }
        if(gamepad2.dpad_left){
            outtake.incrementOutShoPos(-0.002f);
        }

        if (gamepad1.cross) {
            intake.intakeGroundPos();
        }
        
        // Passoff Sequence
        if (gamepad1.triangle) {
            if (!pressed1triangle){
                passoffStage = 1;
                runtime.reset();
                intake.intakePassoffPos();
                closeInClawWithController();
                slides.vertSlidePassoffPos();
                slides.horSlidePassoffPos();
            }
            pressed1triangle = true;
        } else {
            pressed1triangle = false;
        }

        if(passoffStage == 1 && runtime.seconds() > Intake.wristBuffer) {
            intake.intakeAfterPassoffPos();
            outtake.openOutClaw();
            outtake.outtakePassoffPos();
            runtime.reset();
            passoffStage = 2;
        }

        if(passoffStage == 2 && runtime.seconds() > Outtake.clawBuffer) {
            outtake.closeOutClaw();
            openInClawWithController();
            intake.intakeNeutralPos();
            passoffStage = 0;
        }

        if (gamepad1.circle) {
            intake.intakeNeutralPos();
        }
        yDisplacement = cv.getProjection().y;
        if (gamepad1.square) {
            if (!pressed1square){
                intake.setInWristForCV(cv.getDetectedAngle());
                if (cv.getProjection().validProjection){
                    slides.setHorSlidesForCV(yDisplacement);
                }
            }
            pressed1square = true;
        } else {
            pressed1square = false;
        }

        if(gamepad1.right_bumper){
            if(!pressed1rb){
                toggleInClawWithController();
            }
            pressed1rb = true;
        } else {
            pressed1rb = false;
        }

        if(gamepad1.left_bumper){
            if(!pressed1lb) {
                drive.switchState();
            }
            pressed1lb = true;
        } else {
            pressed1lb = false;
        }
        // Can be Commented Out
        if (gamepad2.cross) {
            outtake.outtakePassoffPos();
            slides.vertSlidePassoffPos();
        }

        if (gamepad2.triangle) {
            intake.intakeNeutralPos();
            outtake.outtakePlacePos();
        }
        if (gamepad2.circle) {
            intake.intakeNeutralPos();
            outtake.outtakeSpecimenPos();
        }

//        if(gamepad2.square){
//            slides.horIncrement(-120);
//        }

        if (gamepad2.right_bumper) {
            if (!pressed2rb) {
                outtake.toggleOutClaw();
            }
            pressed2rb = true;
        }
        else {
            pressed2rb = false;
        }

        if (gamepad2.left_bumper) {
            if (!pressed2lb) {
                hangMode = !hangMode;
                if (hangMode) {
                    gamepad2.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                } else {
                    gamepad2.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
                }
            }
            pressed2lb = true;
        } else {
            pressed2lb = false;
        }

        if(gamepad2.square && !onWall){
            if(!pressed2square) {
                slides.vertSlideWallPos();
                runtime.reset();
                wallPosStage = 1;
            }
                pressed2square = true;
        }else{
            pressed2square = false;
        }
        if(wallPosStage == 1 && runtime.seconds() > Outtake.wallPosBuffer){
            outtake.outtakeWallPos();
            onWall = true;
            wallPosStage = 0;
        }
        if(onWall && gamepad2.square) {
            slides.vertSlideWallPickup();
            onWall = false;
            pressed2square = false;
        }
    }
}