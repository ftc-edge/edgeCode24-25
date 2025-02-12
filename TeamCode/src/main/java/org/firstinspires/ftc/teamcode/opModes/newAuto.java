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


    public static double specPosY = -48;
    public static double specPosX = 0;
    public static double hookInY = -24;

    public static double sampleGrabX = 36;
    public static double sampleGrabY = -12;
    public static double sample1X = 48;
    public static double samplePushY = -60;
    public static double sampleGrabIncrementY = 8;


    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, -65, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        outtake = new Outtake(hardwareMap);
        slides = new Slides(hardwareMap);
        waitForStart();

//        outtake.closeOutClaw();
//        outtake.outtakeSpecimenPos();
//        slides.vertSlidesSpecimenPos();

        drive.updatePoseEstimate();
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(specPosX, specPosY))
                .build();

        drive.updatePoseEstimate();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(specPosX, hookInY))
                    .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(sampleGrabX, hookInY))
                                .lineTo(new Vector2d(sampleGrabX, sampleGrabY))
                                        .lineTo(new Vector2d(sample1X, sampleGrabY))
                                                .lineTo(new Vector2d(sample1X, samplePushY))
                                                        .build();

        drive.followTrajectorySequence(traj1);
        //outtake.outtakeHookInPos();
        drive.followTrajectorySequence(traj2);
        drive.followTrajectorySequence(traj3);


    }
}
