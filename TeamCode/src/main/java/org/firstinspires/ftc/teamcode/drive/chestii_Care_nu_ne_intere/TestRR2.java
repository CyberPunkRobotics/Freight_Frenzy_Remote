package org.firstinspires.ftc.teamcode.drive.chestii_Care_nu_ne_intere;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.properties.SampleMecanumDrive;


@Autonomous(name = "TestRR2")
public class TestRR2 extends LinearOpMode {
    SampleMecanumDrive drive;

    void leftCase() {
        Pose2d currentPose = drive.getPoseEstimate();
            Trajectory traj1 = drive.trajectoryBuilder(currentPose)
                    .splineTo(new Vector2d(50,50),0)
                    .build();

         drive.followTrajectory(traj1);

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
