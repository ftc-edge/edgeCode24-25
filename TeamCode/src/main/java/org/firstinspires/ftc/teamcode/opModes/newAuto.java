package org.firstinspires.ftc.teamcode.opModes;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.Slides;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Outtake;
import org.firstinspires.ftc.teamcode.components.Auto;


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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class newAuto extends LinearOpMode {
    Drive drive;
    Slides slides;
    Intake intake;
    Outtake outtake;

    Auto auto;

    Servo lArm, rArm;


    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, -65, Math.toRadians(auto.startingHeading));
        drive.setPoseEstimate(startPose);

        waitForStart();

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        slides = new Slides(hardwareMap);
        auto = new Auto();

//        outtake.closeOutClaw();
//        outtake.outtakeSpecimenPos();
//        slides.vertSlidesSpecimenPos();


        drive.updatePoseEstimate();
        TrajectorySequence smallForward = drive.trajectorySequenceBuilder(new Pose2d(auto.specPosX, auto.specPosY, auto.startingHeading))
                .forward(auto.smallForwardAmount)
                .build();

        TrajectorySequence initTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(auto.specPosX, auto.initTrajY))
                                .build();

        intake.intakeNeutralPos();
        intake.update();

        outtake.closeOutClaw();
        outtake.outtakeSpecimenPos();
        outtake.update();
        slides.vertSlidesSpecimenPos();

        drive.updatePoseEstimate();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(auto.specPosX, auto.hookInY))
                .build();
        
        drive.followTrajectorySequence(initTraj);
        drive.updatePoseEstimate();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(auto.specPosX, auto.specPosY))
                .build();
        drive.followTrajectorySequence(traj1);
        drive.updatePoseEstimate();
        slides.vertSlideHookInPos();

        drive.updatePoseEstimate();
        TrajectorySequence buffer = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .waitSeconds(5)
                .build();
        
        drive.followTrajectorySequence(buffer);
        drive.updatePoseEstimate();

        outtake.openOutClaw();
        outtake.update();
        slides.vertSlideWallPos();

        drive.updatePoseEstimate();
        traj2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(auto.specPosX, auto.hookInY))
                .build();
        
        drive.followTrajectorySequence(traj2);
        
        drive.updatePoseEstimate();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(auto.sampleGrabX, auto.hookInY))
                .lineTo(new Vector2d(auto.sampleGrabX, auto.sampleGrabY))
                .lineTo(new Vector2d(auto.sample1X, auto.sampleGrabY))
                .lineTo(new Vector2d(auto.sample1X, auto.samplePushY))
                .lineTo(new Vector2d(auto.sample1X, auto.sampleGrabY))
                .lineTo(new Vector2d(auto.sample1X + auto.sampleGrabIncrementX, auto.sampleGrabY))
                .lineTo(new Vector2d(auto.sample1X + auto.sampleGrabIncrementX, auto.samplePushY))
                .build();

        drive.followTrajectorySequence(traj3);

        drive.updatePoseEstimate();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(auto.sample1X, auto.mitchellPosY))
                .build();

        drive.followTrajectorySequence(traj4);
        drive.updatePoseEstimate();

        outtake.outtakeWallPos();
        outtake.update();
        drive.updatePoseEstimate();
        buffer = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .waitSeconds(5)
                .build();
        
        drive.followTrajectorySequence(buffer);

        outtake.closeOutClaw();
        outtake.update();
        slides.vertSlideWallPickup();

        outtake.outtakeSpecimenPos();
        outtake.update();
        slides.vertSlidesSpecimenPos();

        drive.updatePoseEstimate();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(auto.specPosX, auto.hookInY))
                .lineTo(new Vector2d(auto.specPosX, auto.specPosY))
                .build();
        drive.followTrajectorySequence(traj5);
        drive.updatePoseEstimate();

        slides.vertSlideHookInPos();

        drive.updatePoseEstimate();
        buffer = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .waitSeconds(5)
                .build();
        drive.followTrajectorySequence(buffer);

        drive.updatePoseEstimate();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(auto.sample1X, auto.mitchellPosY))
                //.lineTo(new Vector2d(auto.sample1X, auto.mitchellPosY))
                .build();
        drive.followTrajectorySequence(traj6);

        slides.resetSlides();
        }
    }

