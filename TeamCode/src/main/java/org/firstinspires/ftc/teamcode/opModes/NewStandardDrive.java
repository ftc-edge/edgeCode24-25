package org.firstinspires.ftc.teamcode.opModes;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.Slides;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Outtake;

import java.util.*;


@TeleOp()
public class NewStandardDrive extends OpMode {

    Drive drive;
    Slides slides;
    Intake intake;
    Outtake outtake;

    private ElapsedTime runtime = new ElapsedTime();
    boolean passoffTimer = false;

    boolean pressed2x = false;
    boolean pressed1x = false;

    // TODO: Hang Mode

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        slides = new Slides(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
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
        slides.vertIncrement(15 * (int) (gamepad2.left_trigger - gamepad2.right_trigger));
        slides.horIncrement(10 * (int) gamepad2.left_stick_y);
        intake.update();
        outtake.update();
    }

    private void telemetry(){
        telemetry.addData("time", getTime());
        telemetry.addData("vertSlidePos", slides.getVertSlidePos());
        telemetry.addData("horSlidePos", slides.getHorSlidePosition());
        telemetry.addData("outWrist", outtake.outWristPos);
        telemetry.addData("outSho", outtake.outShoulderPos);
        telemetry.addData("inWrist", intake.wristServoPos);
        telemetry.addData("inLArm", intake.lArmServoPos);
        
        telemetry.update();
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

        if (gamepad1.a) {
            intake.intakeGroundPos();
        }
        if (gamepad1.y) {
            passoffTimer = true;
            runtime.reset();
            intake.intakePassoffPos();
        }
        if(passoffTimer && runtime.seconds() > 0.6) {
            passoffTimer = false;
            intake.intakeAfterPassoffPos();
        }
        if (gamepad1.b) {
            intake.intakeNeutralPos();
        }
        if (gamepad1.x) {
            if (!pressed1x){
                intake.toggleInClaw();
            }
            pressed1x = true;
        } else {
            pressed1x = false;
        }

        if (gamepad2.b) {
            outtake.outtakePassoffPos();
        }
        if (gamepad2.y) {
            outtake.outtakePlacePos();
        }
        if (gamepad2.a) {
            outtake.outtakeSpecimenPos();
        }
        if (gamepad2.x) {
            if (!pressed2x) {
                outtake.toggleOutClaw();
            }
            pressed2x = true;
        }
        else {
            pressed2x = false;
        }

    }
}