package org.firstinspires.ftc.teamcode.opModes;

import static java.lang.Math.max;
import static java.lang.Math.min;

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
public class newAuto extends LinearOpMode{
    Drive drive;
    Slides slides;
    Intake intake;
    Outtake outtake;

    Auto auto;





    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, -65, Math.toRadians(auto.startingHeading));
        drive.setPoseEstimate(startPose);
        outtake = new Outtake(hardwareMap);
        slides = new Slides(hardwareMap);
        auto = new Auto();
        waitForStart();

//        outtake.closeOutClaw();
//        outtake.outtakeSpecimenPos();
//        slides.vertSlidesSpecimenPos();

        drive.updatePoseEstimate();
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(auto.specPosX, auto.specPosY))
                .build();

        drive.updatePoseEstimate();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(auto.specPosX, auto.hookInY))
                    .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(auto.sampleGrabX, auto.hookInY))
                                .lineTo(new Vector2d(auto.sampleGrabX, auto.sampleGrabY))
                                        .lineTo(new Vector2d(auto.sample1X, auto.sampleGrabY))
                                                .lineTo(new Vector2d(auto.sample1X, auto.samplePushY))
                                                        .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(auto.sample1X + auto.sampleGrabIncrementX, auto.sampleGrabY))
                                .lineTo(new Vector2d(auto.sample1X + auto.sampleGrabIncrementX, auto.samplePushY))
                                        .lineTo(new Vector2d(auto.sample1X + auto.sampleGrabIncrementX,  auto.sampleGrabY))
                                                .lineTo(new Vector2d(auto.sample1X + auto.sampleGrabIncrementX + auto.sampleGrabIncrementX, auto.sampleGrabY))
                                                        .lineTo(new Vector2d(auto.sample1X + auto.sampleGrabIncrementX + auto.sampleGrabIncrementX, auto.samplePushY))
                                                                .build();
        drive.followTrajectorySequence(traj1);
        //outtake.outtakeHookInPos();
        drive.followTrajectorySequence(traj2);
        drive.followTrajectorySequence(traj3);
        drive.followTrajectorySequence(traj4);


    }
}
