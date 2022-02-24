package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.properties.SampleMecanumDrive;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


@TeleOp(name = "TestGenFiles")
public class TestGenFiles extends LinearOpMode {

    SampleMecanumDrive drive;
    ElapsedTime runtime = new ElapsedTime();
    int noOfTrajectories = 0;
    FileWriter myWriter = new FileWriter(new File("/storage/emulated/0/AutoFiles", "Test.java"));

    public TestGenFiles() throws IOException {
    }


    void writeFiles(boolean reverse) throws IOException {

        Pose2d pose = drive.getPoseEstimate();
        noOfTrajectories++;
        myWriter.write("drive.updatePoseEstimate();\n" +
                "        currentPose = drive.getPoseEstimate();\n" +
                "       Trajectory traj" + noOfTrajectories + " = drive.trajectoryBuilder(currentPose," +
                (reverse ? "true" : "false") +  ")\n" +
                "                .splineTo(new Vector2d(" + pose.getX() +  ", " + pose.getY() + "), " +
                pose.getHeading() +  ")\n" +
                "                .build();\n" +
                "\n" +
                "        drive.followTrajectory(traj" + noOfTrajectories + ");\n");


    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        try {
            myWriter.write("void roadRunnerSplines() {\ndrive.updatePoseEstimate();\n" +
                    "        Pose2d currentPose;");
        } catch (IOException e) {
            e.printStackTrace();
        }

        runtime.reset();
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double sF = (forward + strafe + rotate)/2;
            double sS = (forward - strafe + rotate)/2;
            double dF = (forward - strafe - rotate)/2;
            double dS = (forward + strafe - rotate)/2;

            drive.setMotorPowers(sF, sS, dS, dF);

            telemetry.addData("Pose X: ", drive.getPoseEstimate().getX());
            telemetry.addData("Pose Y: ", drive.getPoseEstimate().getY());
            telemetry.addData("Pose Heading: ", drive.getPoseEstimate().getHeading());
            telemetry.update();


            if (gamepad1.x && runtime.seconds() > 1) {
                runtime.reset();
                try {
                    writeFiles(false);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            if (gamepad1.y && runtime.seconds() > 1) {
                runtime.reset();
                try {
                    writeFiles(true);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

        }
        try {
            myWriter.write("\n}");
            myWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

//            System.out.println("Current dir:" + Arrays.toString(new File(".").listFiles()));

    }
}
