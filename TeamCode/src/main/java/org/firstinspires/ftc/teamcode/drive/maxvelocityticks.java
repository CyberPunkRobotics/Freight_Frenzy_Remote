package org.firstinspires.ftc.teamcode.drive;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
@Disabled
@Autonomous(name="maxvelocityticks")
public class maxvelocityticks extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "stangaFata");
        leftRear = hardwareMap.get(DcMotorEx.class, "stangaSpate");
        rightRear = hardwareMap.get(DcMotorEx.class, "dreaptaSpate");
        rightFront = hardwareMap.get(DcMotorEx.class, "dreaptaFata");
        waitForStart();
        while (opModeIsActive()) {
            currentVelocity = motor.getVelocity();
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}
