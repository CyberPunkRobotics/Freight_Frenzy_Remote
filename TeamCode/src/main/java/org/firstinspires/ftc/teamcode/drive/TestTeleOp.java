package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double sS = -strafe - forward - rotate;
            double dF = strafe + forward - rotate;
            double sF = strafe - forward - rotate;
            double dS = -strafe + forward - rotate;


            drive.setMotorPowers(sF, sS, dS, dF);

            telemetry.addData("Heading: ", drive.getRawExternalHeading());
            telemetry.update();
        }
    }
}
