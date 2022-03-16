package org.firstinspires.ftc.teamcode.drive.chestii_Care_nu_ne_intere;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.properties.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.properties.StandardTrackingWheelLocalizer;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        String pozitie = "shipping_hub";
        ElapsedTime time = new ElapsedTime();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //5.204114251977755,-63.62056043033013


        drive.setPoseEstimate(new Pose2d(5.5674211343158015,-64.5442651728134));
        drive.setExternalHeading(Math.toRadians(0));


        //localizer.poseEstimate

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    ));

//            if (drive.culoareSpate.red()>=80 && drive.culoareSpate.green()>=150 && drive.culoareSpate.blue() >= 150 && drive.culoareSpate.alpha()>=90){
//                //robot.setPoseEstimate(new Pose2d());
//                if(pozitie == "shipping_hub" && time.time() > 1){
//                    pozitie = "warehouse";
//                    time.reset();
//                }
//                else if(pozitie == "warehouse" && time.time() > 1) {
//                    pozitie = "shipping_hub";
//                    time.reset();
//                }
//                drive.setPoseEstimate(new Pose2d(28.594453496583984,-62.96759694447507));
//            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
