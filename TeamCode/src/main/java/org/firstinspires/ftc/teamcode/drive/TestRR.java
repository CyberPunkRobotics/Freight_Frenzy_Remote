package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TestRR")
public class TestRR extends LinearOpMode {
    SampleMecanumDrive drive;

    void roadRunnerSplines() {
        drive.updatePoseEstimate();
        Pose2d currentPose;drive.updatePoseEstimate();
        currentPose = drive.getPoseEstimate();
        Trajectory traj1 = drive.trajectoryBuilder(currentPose,false)
                .splineTo(new Vector2d(22.572949636176517, -28.261237605004965), 3.165407359600067)
                .build();

        drive.followTrajectory(traj1);
        drive.updatePoseEstimate();
        currentPose = drive.getPoseEstimate();
        Trajectory traj2 = drive.trajectoryBuilder(currentPose,false)
                .splineTo(new Vector2d(0.9483601531841619, -63.67024728543801), 4.717629675065176)
                .splineTo(new Vector2d(25.57507839043114, -79.77881567506564), 0.04333332180976868)
                .build();

        drive.followTrajectory(traj2);
//        drive.updatePoseEstimate();
//        currentPose = drive.getPoseEstimate();
//        Trajectory traj3 = drive.trajectoryBuilder(currentPose,false)
//
//                .build();
//
//        drive.followTrajectory(traj3);

    }

    void leftCase() {

        Pose2d currentPose = drive.getPoseEstimate();
        Trajectory traj1 = drive.trajectoryBuilder(currentPose)
                .forward(5)
                .build();

        drive.followTrajectory(traj1);

        drive.updatePoseEstimate();
        currentPose = drive.getPoseEstimate();
        Trajectory traj2 = drive.trajectoryBuilder(currentPose)
                .strafeLeft(16)
                .build();
        drive.followTrajectory(traj2);

        drive.turn(Math.toRadians(180));

        drive.updatePoseEstimate();
        currentPose = drive.getPoseEstimate();

        Trajectory traj3 = drive.trajectoryBuilder(currentPose, true)
                .splineTo(new Vector2d(15,-25),0)
                .splineTo(new Vector2d(22, -25), 0)
                .build();
        drive.followTrajectory(traj3);


        drive.updatePoseEstimate();
        currentPose = drive.getPoseEstimate();

        Trajectory traj5 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(10,-80),0)
                .splineTo (new Vector2d(22, -80), 0)
                .build();
        drive.followTrajectory(traj5);

    }
    @Override
    public void runOpMode() throws InterruptedException{
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d pozitieStart = new Pose2d(0,0,0);
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(pozitieStart);

//        Trajectory traj1 = drive.trajectoryBuilder(pozitieStart)
//                .splineTo(new Vector2d(30,20),0)
//                .build();
//
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .strafeRight(5)
//                .build();


        waitForStart();

        if(isStopRequested()){
            return;
        }

        roadRunnerSplines();
//        leftCase();
//        drive.followTrajectory(traj1);
//        drive.followTrajectory(traj2);
    }
}
