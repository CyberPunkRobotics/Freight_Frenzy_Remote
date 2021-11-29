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

        Pose2d currentPose = drive.getPoseEstimate();
        Trajectory traj1 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(17 ,20),Math.PI)
                .build();

        drive.followTrajectory(traj1);

        drive.updatePoseEstimate();
        currentPose = drive.getPoseEstimate();
        Trajectory traj2 = drive.trajectoryBuilder(currentPose)
                .back(5)
                .build();
        drive.followTrajectory(traj2);

        drive.updatePoseEstimate();
        currentPose = drive.getPoseEstimate();

        Trajectory traj5 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(13, -30),0)
                .splineTo (new Vector2d(22, -30), 0)
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

        leftCase();
//        drive.followTrajectory(traj1);
//        drive.followTrajectory(traj2);
    }
}
