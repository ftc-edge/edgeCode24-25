package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class blueSideAuto extends LinearOpMode{
    private DcMotorEx slideMotor;
    public void runOpMode() throws InterruptedException {

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(1600);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, -65, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .back(32)
                .waitSeconds(2)
                .splineTo(new Vector2d(5,-43), Math.toRadians(0))
                .waitSeconds(1)
                .splineTo(new Vector2d(30, -20), Math.toRadians(90))
                .splineTo(new Vector2d(46, -6), Math.toRadians(90))
//                .back(42)
//                .forward(42)
//                .strafeRight(12)
//                .back(42)
                .build();

        drive.followTrajectorySequence(traj);



    }

    public void specimenPos(){
        slideMotor.setTargetPosition(1600);
        slideMotor.setPower(0.6);
    }

    public void retract(){
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(0.4);
    }

}
