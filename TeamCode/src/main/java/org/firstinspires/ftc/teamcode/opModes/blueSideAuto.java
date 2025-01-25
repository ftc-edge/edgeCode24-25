package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.opModes.standardDrive;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class blueSideAuto extends LinearOpMode{
    private DcMotorEx slideMotor;
    private DcMotorEx underSlide;
    private Servo outShoulder;
    public void runOpMode() throws InterruptedException {

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(2075);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        underSlide = hardwareMap.get(DcMotorEx.class, "underSlide");
        underSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        underSlide.setDirection(DcMotorEx.Direction.REVERSE);
        underSlide.setTargetPosition(2075);
        underSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        outShoulder = hardwareMap.get(Servo.class, "outShoulder");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, -65, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

//        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
//                .back(28)
//                .waitSeconds(2)
//                .forward(14)
//                .turn(Math.toRadians(180))
//                //.splineTo(new Vector2d(52, -58), Math.toRadians(90))
//                .lineTo(new Vector2d(52, -56))
//                .build();

        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(startPose)
                .back(15)
                .build();

        TrajectorySequence wait = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(5)
                .build();


        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .back(27.5)
                .waitSeconds(2)
                //.splineTo(new Vector2d(5,-50), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(5, -50), Math.toRadians(0))
                .waitSeconds(1)
                //.splineTo(new Vector2d(52, -58), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(52, -56, Math.toRadians(90)), Math.toRadians(0))
//                .back(42)
//                .forward(42)
//                .strafeRight(12)
//                .back(42)
                .build();

        drive.followTrajectorySequence(backTraj);
        specimenPos();
        outShoulder.setPosition(0.1);
        drive.followTrajectorySequence(wait);

        //drive.followTrajectorySequence(traj);



    }

    public void specimenPos() {
        slideMotor.setTargetPosition(2075);
        underSlide.setTargetPosition(2075);
        slideMotor.setPower(0.5);
        underSlide.setPower(0.5);
        outShoulder.setPosition(0.1);
    }

    public void retract(){
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(0.4);
    }

}
