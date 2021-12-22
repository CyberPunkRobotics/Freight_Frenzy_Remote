package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TestRR2")
public class TestRR2 extends LinearOpMode {
    SampleMecanumDrive drive;

    void leftCase() {

            drive.updatePoseEstimate();
            Pose2d currentPose;drive.updatePoseEstimate();
            currentPose = drive.getPoseEstimate();
            Trajectory traj1 = drive.trajectoryBuilder(currentPose,false)
                    .splineTo(new Vector2d(18.162388753361064, -27.96293036149466), 0.14888906083069742)
                    .build();

            drive.followTrajectory(traj1);
            drive.updatePoseEstimate();
            currentPose = drive.getPoseEstimate();
            Trajectory traj2 = drive.trajectoryBuilder(currentPose,false)
                    .splineTo(new Vector2d(-1.109289057955717, -68.91857577347113), 4.703333377848388)
                    .build();

            drive.followTrajectory(traj2);
            drive.updatePoseEstimate();
            currentPose = drive.getPoseEstimate();
            Trajectory traj3 = drive.trajectoryBuilder(currentPose,false)
                    .splineTo(new Vector2d(27.588412346860654, -82.00947073330613), 0.0277776715811342)
                    .build();

            drive.followTrajectory(traj3);
            drive.updatePoseEstimate();
            currentPose = drive.getPoseEstimate();
            Trajectory traj4 = drive.trajectoryBuilder(currentPose,false)
                    .splineTo(new Vector2d(43.72590844145853, -107.25763105805525), 0.17000007233582437)
                    .build();

            drive.followTrajectory(traj4);




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

        leftCase();
//        drive.followTrajectory(traj1);
//        drive.followTrajectory(traj2);
    }
}
