package org.firstinspires.ftc.teamcode.drive.chestii_Care_nu_ne_intere;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.properties.SampleMecanumDrive;

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
            double sS = forward + rotate - strafe;
            double sF = forward + rotate + strafe;
            double dF = -forward + rotate - strafe;
            double dS = -forward + rotate + strafe;


            drive.setMotorPowers(sF, sS, dS, dF);

            telemetry.addData("Heading: ", Math.toDegrees(drive.getRawExternalHeading()));
            telemetry.update();
        }
    }
}
