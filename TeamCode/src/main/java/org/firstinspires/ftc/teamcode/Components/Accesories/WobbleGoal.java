/**
 * This is the program for moving the wobble goal arm up and down
 * @author Sid
 * @version 2.0
 * @since 2020-11-08
 * @status teleop working
 */

package org.firstinspires.ftc.teamcode.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoal {

    public enum Position {
        REST, GRAB, RAISE, RELEASE, DROP, STARTOFTELEEOP
    }

    //declaring the op mode
    private LinearOpMode op;

    protected DcMotor wobbleGoalMotor = null;

    protected Servo wobbleGoalServo = null;

    protected Servo wobbleGoalServoClaw = null;

    private final int ticksForREST = 0;
    private final int ticksForGRAB = 540;
    private final int ticksForRAISE = 300;
    private final int ticksForAutonomousDrop = 950;
    private final int ticksForSTARTOFTELEEOP = 200;
    private final double wobbleGoalSpeed = 0.3;
    private final double wobbleGoalSpeedDrop = 0.5;

    public WobbleGoal(LinearOpMode opMode) {
        //setting the opmode
        this.op = opMode;

        //getting the motor & servo from the hardware map
        wobbleGoalMotor = (DcMotor) opMode.hardwareMap.get("wobbleGoalMotor");
        wobbleGoalServo = op.hardwareMap.servo.get("WobbleGoalServo");
        wobbleGoalServoClaw = op.hardwareMap.servo.get("wobbleGoalServoClaw");
        wobbleGoalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleGoalMotor.setDirection(DcMotor.Direction.FORWARD);
        wobbleGoalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleGoalServo.setPosition(0);
        wobbleGoalServoClaw.setPosition(1);

    }

    //tells motor to go to a specified position based on ticks(i)
    public void goToPosition(Position p) {
        int i = 0;
        if (p == Position.REST) {
            i = ticksForREST;
        } else if (p == Position.GRAB) {
            i = ticksForGRAB;
        } else if (p == Position.RAISE) {
            i = ticksForRAISE;
        } else if (p == Position.DROP) {
            i = ticksForAutonomousDrop;
        } else {
            op.telemetry.addData("IQ Lvl", "0.00");
            op.telemetry.update();
            op.sleep(2000);
        }

        wobbleGoalMotor.setTargetPosition(i);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (p== Position.DROP){
            wobbleGoalMotor.setPower(wobbleGoalSpeedDrop);
        } else{
            wobbleGoalMotor.setPower(wobbleGoalSpeed);
        }
//        op.sleep(100);
        op.telemetry.addData("Wobble Goal", "Position:" + wobbleGoalMotor.getCurrentPosition() + "-->" + i);
        op.telemetry.update();
//        op.sleep(2000);
    }

    //TODO: Siddharth & Aiden, can this folded into above gotoPosition method?
    public void teleopStartPosition(){
        wobbleGoalMotor.setTargetPosition(ticksForSTARTOFTELEEOP);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setPower(wobbleGoalSpeed);
    }

    public void printCurrentLocation(){
        op.telemetry.addData("Wobble Goal ", "Current Position:" + wobbleGoalMotor.getCurrentPosition());
        op.telemetry.update();
//        op.sleep(2000);
    }

    //stops the motor
    public void stop() {
        wobbleGoalMotor.setPower(0);
    }

    // moves the wobble goal servo
    public void moveWobbleGoalServo (boolean direction){
        if (direction){
            wobbleGoalServo.setPosition(1.0);
        } else {
            wobbleGoalServo.setPosition(0.0);
        }
        op.telemetry.addData(" Wobble Goal Position: ", direction);
        op.telemetry.update();
        op.sleep(500);
    }

    public void moveWobbleGoalClaw(boolean direction) {
        if (direction){
            wobbleGoalServoClaw.setPosition(1);
            op.sleep(1000);
            op.telemetry.addData(" Wobble Goal Claw: ", "open");
        } else {
            wobbleGoalServoClaw.setPosition(0.5);
            op.sleep(1000);
            op.telemetry.addData(" Wobble Goal Claw: ", "closed");

        }
        op.telemetry.update();
    }
}

