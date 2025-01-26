package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opModes.standardDrive;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
PROCEDURE
 - Move Back, to Hook
 - Rotate slide up, Rotate arm & wrist
 - Drive forward, decrease grip on block
 - Release grip, rotate arm back
 - Drive to human player area (Puts in specimen)
 - Return to position and repeat
 */

@Autonomous
public class blueSideAuto extends LinearOpMode {
    private DcMotorEx slideMotor;
    private DcMotorEx underSlide;
    private Servo outShoulder;
    private Servo outWrist;

    private Servo outFinger1;
    private Servo outFinger2;
    private Servo clawElbow;
    private Servo clawShoulder;
    private Servo clawFinger1;
    private Servo clawFinger2;
    private Servo clawWrist;

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        underSlide = hardwareMap.get(DcMotorEx.class, "underSlide");
//        underSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        underSlide.setDirection(DcMotorEx.Direction.REVERSE);
//        underSlide.setTargetPosition(0);
//        underSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        outShoulder = hardwareMap.get(Servo.class, "outShoulder");
        outWrist = hardwareMap.get(Servo.class, "outWrist");
        outFinger1 = hardwareMap.get(Servo.class, "outFinger1");
        outFinger2 = hardwareMap.get(Servo.class, "outFinger2");

        clawShoulder = hardwareMap.get(Servo.class, "clawShoulder");
        clawElbow = hardwareMap.get(Servo.class, "clawElbow");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        clawFinger1 = hardwareMap.get(Servo.class, "clawFinger1");
        clawFinger2 = hardwareMap.get(Servo.class, "clawFinger2");

        telemetry.addData("position", slideMotor.getCurrentPosition());
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, -65, Math.toRadians(-90));
        Pose2d currentPose = drive.getPoseEstimate();

        drive.setPoseEstimate(startPose);

        waitForStart();

        outFinger1.setPosition(0.3);
        outFinger2.setPosition(0.3);

        clawWrist.setPosition(0.1);
        clawElbow.setPosition(0.69);
        clawShoulder.setPosition(0.434);
        clawFinger1.setPosition(0.15);
        clawFinger2.setPosition(0.15);

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            drive.update();

            currentPose = drive.getPoseEstimate();

//        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
//                .back(28)
//                .waitSeconds(2)
//                .forward(14)
//                .turn(Math.toRadians(180))
//                //.splineTo(new Vector2d(52, -58), Math.toRadians(90))
//                .lineTo(new Vector2d(52, -56))
//                .build();

            TrajectorySequence backTraj = drive.trajectorySequenceBuilder(startPose)
                    .back(17)
                    .build();

            TrajectorySequence backTraj2 = drive.trajectorySequenceBuilder(new Pose2d(0, -48, Math.toRadians(-90)))
                    .back(10)
                    .build();

            TrajectorySequence hook = drive.trajectorySequenceBuilder(new Pose2d(0, -38, Math.toRadians(-90)))
                    .forward(21)
                    .build();


            TrajectorySequence smallBuffer = drive.trajectorySequenceBuilder(currentPose)
                    .waitSeconds(0.1)
                    .build();

            TrajectorySequence buffer = drive.trajectorySequenceBuilder(currentPose)
                    .waitSeconds(0.3)
                    .build();

            TrajectorySequence bufferForSwing = drive.trajectorySequenceBuilder(new Pose2d(0, -48, Math.toRadians(-90)))
                    .waitSeconds(0.9)
                    .build();

            TrajectorySequence bufferAfterHook = drive.trajectorySequenceBuilder(new Pose2d(0, -55, Math.toRadians(-90)))
                    .waitSeconds(0.9)
                    .build();


            TrajectorySequence largeBuffer = drive.trajectorySequenceBuilder(currentPose)
                    .waitSeconds(1)
                    .build();

            TrajectorySequence mitchellTraj = drive.trajectorySequenceBuilder(new Pose2d(0, -55, Math.toRadians(-90)))
                    .splineToSplineHeading(new Pose2d(5, -50), Math.toRadians(0))
                    .waitSeconds(0.1)
                    .splineToSplineHeading(new Pose2d(55, -56, Math.toRadians(90)), Math.toRadians(0))
                    .back(12)
                    .build();

            TrajectorySequence mitchellPickup = drive.trajectorySequenceBuilder(currentPose)
                    .back(3)
                    .build();

            TrajectorySequence placeTraj = drive.trajectorySequenceBuilder(currentPose)
                    .splineToSplineHeading(new Pose2d(0, -42, Math.toRadians(-90)), Math.toRadians(0))
                    .build();

            TrajectorySequence firstBlockTraj = drive.trajectorySequenceBuilder(currentPose)
                    .lineToLinearHeading(new Pose2d(52, -42, Math.toRadians(90)))
                    .build();
            TrajectorySequence secondBlockTraj = drive.trajectorySequenceBuilder(currentPose)
                    .lineToLinearHeading(new Pose2d(64, -42, Math.toRadians(90)))
                    .build();

            drive.followTrajectorySequence(backTraj);
            //PLACING SPECIMEN
            specimenPos();
            swingOut();
            drive.followTrajectorySequence(bufferForSwing);
            drive.followTrajectorySequence(backTraj2);
            outShoulder.setPosition(0.135);
            drive.followTrajectorySequence(hook);
            openFinger();
            outShoulder.setPosition(0.1);
            drive.followTrajectorySequence(bufferAfterHook);
            outtakePassoff();
            retract();
            drive.followTrajectorySequence(mitchellTraj);
            /*
            openFinger();]]
            retract();
            //HEADING TOWARDS MITCHELL
            drive.followTrajectorySequence(buffer);
            drive.followTrajectorySequence(mitchellTraj);
            wallPickupPos();
            drive.followTrajectorySequence(largeBuffer);
            shoulderWristPickup();
            drive.followTrajectorySequence(mitchellPickup);
            drive.followTrajectorySequence(buffer);
            closeFinger();
            drive.followTrajectorySequence(buffer);
            //SPECIMEN PLACING TWO
            drive.followTrajectorySequence(placeTraj);
            specimenPos();
            outShoulder.setPosition(0.15);
            drive.followTrajectorySequence(hook);
            openFinger();
            retract();
            //HEADING TOWARDS BLOCK PICKUP
            drive.followTrajectorySequence(firstBlockTraj);
            drive.followTrajectorySequence(buffer);
            //BLOCK PICKUP PROCESS
            intakeSetup();
            drive.followTrajectorySequence(smallBuffer);
            closeIntakeFingers();
            drive.followTrajectorySequence(smallBuffer);
            intakePassoff();
            drive.followTrajectorySequence(smallBuffer);
            outtakePassoff();
            drive.followTrajectorySequence(smallBuffer);
            closeFinger();
            drive.followTrajectorySequence(smallBuffer);
            openIntakeFingers();
            mitchellDrop();
            drive.followTrajectorySequence(smallBuffer);
            openFinger();
            drive.followTrajectorySequence(largeBuffer);
            //MITCHELL PICKUP PT.2
            wallPickupPos();
            drive.followTrajectorySequence(buffer);
            shoulderWristPickup();
            drive.followTrajectorySequence(mitchellPickup);
            drive.followTrajectorySequence(buffer);
            closeFinger();
            //PLACING PT.3
            drive.followTrajectorySequence(placeTraj);
            specimenPos();
            outShoulder.setPosition(0.15);
            drive.followTrajectorySequence(hook);
            openFinger();
            retract();
            //BLOCK PT.2
            drive.followTrajectorySequence(buffer);
            drive.followTrajectorySequence(secondBlockTraj);
            drive.followTrajectorySequence(buffer);
            //PICKUP PT.2
            intakeSetup();
            drive.followTrajectorySequence(smallBuffer);
            closeIntakeFingers();
            drive.followTrajectorySequence(smallBuffer);
            intakePassoff();
            drive.followTrajectorySequence(smallBuffer);
            outtakePassoff();
            drive.followTrajectorySequence(smallBuffer);
            closeFinger();
            drive.followTrajectorySequence(smallBuffer);
            openIntakeFingers();
            mitchellDrop();
            drive.followTrajectorySequence(smallBuffer);
            openFinger();
            drive.followTrajectorySequence(largeBuffer);
            //MITCHELL PT.3
            wallPickupPos();
            drive.followTrajectorySequence(buffer);
            shoulderWristPickup();
            drive.followTrajectorySequence(mitchellPickup);
            drive.followTrajectorySequence(buffer);
            //PLACING PT.4
            drive.followTrajectorySequence(placeTraj);
            specimenPos();
            outShoulder.setPosition(0.15);
            drive.followTrajectorySequence(hook);
            openFinger();
            retract();
            //PARK
            drive.followTrajectorySequence(mitchellTraj);
            */

//    public void loop(){
//        telemetry.addData("slideMotorPosition", slideMotor.getCurrentPosition());
//        telemetry.addData("underSLidePosition", underSlide.getCurrentPosition());
//        telemetry.update();
//    }
        return;

        }
    }

    private void specimenPos() {
        runtime.reset();
        slideMotor.setPower(0.5);
        while ((runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: Elapsed", runtime.seconds());
            telemetry.update();
        }
        slideMotor.setPower(0);
    }

    private void swingOut(){
        outShoulder.setPosition(0.1);
        outWrist.setPosition(0.22);
    }

    public void retract() {
        runtime.reset();
        slideMotor.setPower(-0.5);
        while ((runtime.seconds() < 0.85)) {
            telemetry.addData("Path", "Leg 3: Elapsed", runtime.seconds());
            telemetry.update();
        }
        slideMotor.setPower(0);
    }

    public void closeFinger(){
        outFinger1.setPosition(0.3);
        outFinger2.setPosition(0.3);
    }

    public void openFinger(){
        outFinger1.setPosition(0);
        outFinger2.setPosition(0.6);
    }

    public void wallPickupPos(){
        openFinger();
        slideMotor.setTargetPosition(537);
        slideMotor.setPower(0.1);
    }

    public void shoulderWristPickup(){
        outShoulder.setPosition(0.257);
        outWrist.setPosition(0.972);
    }

    public void intakeSetup(){
        clawFinger1.setPosition(0.8);
        clawFinger2.setPosition(0.4);
        clawElbow.setPosition(0.2);
        clawShoulder.setPosition(0.33);
    }
    public void closeIntakeFingers(){
        clawFinger1.setPosition(0.43);
        clawFinger2.setPosition(0.95);
    }
    public void openIntakeFingers(){
        clawFinger1.setPosition(0.8);
        clawFinger2.setPosition(0.4);
    }
    public void intakePassoff(){
        clawWrist.setPosition(0.1);
        clawElbow.setPosition(0.69);
        clawShoulder.setPosition(0.434);
    }
    public void outtakePassoff(){
        outShoulder.setPosition(0.593);
        outWrist.setPosition(0.479);
    }
    public void mitchellDrop(){
        outShoulder.setPosition(0.257);
        outWrist.setPosition(0.972);
    }
}
