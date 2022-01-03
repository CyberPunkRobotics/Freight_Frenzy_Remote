package org.firstinspires.ftc.teamcode.drive.properties;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
