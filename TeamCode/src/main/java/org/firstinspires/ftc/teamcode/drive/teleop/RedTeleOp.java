package org.firstinspires.ftc.teamcode.drive.teleop;

import android.app.Activity;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.properties.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.properties.StandardTrackingWheelLocalizer;

import static org.firstinspires.ftc.robotcore.internal.android.dx.dex.code.StdCatchBuilder.build;

//package org.firstinspires.ftc.teamcode.drive;
  //      package org.firstinspires.ftc.teamcode.drive;

@TeleOp(name = "RedTeleOp", group = "MecanumBot")
public class RedTeleOp extends LinearOpMode {

    private SampleMecanumDrive robot = null;
    private final double DELAY = 0.2;
    public ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        robot = new SampleMecanumDrive(hardwareMap);

        //encodere pt ridicare brat
        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //robot in pozitie de start
        double xstart,ystart;
        xstart = -41.862855949849056;
        ystart = -63.11163142092735;
        robot.setPoseEstimate(new Pose2d(xstart,ystart));

        //brat in pozitia initiala
        //robot.PivotBrat.setPosition(0.75);

        //contor tickuri ridicare/coborare brat
        int ticks;

        //pt determinare pozitie robot (la inceput e inafara warehouse-ului)
        String pozitie = "shipping_hub";
        String nivel_brat = "pozitie_initiala";

        //telemetry pe dashboard
        Telemetry dashboardTelemetry;
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.cleste.setPosition(0.6);
        robot.PivotBrat.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {

            //contorizare tick-uri
            ticks = robot.ridicareBrat.getCurrentPosition();

            runtime.reset();


            /** GAMEPAD 1 */


            //Miscarea sasiului

            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            double direction = Math.atan2(x, y) - Math.toRadians(robot.getAngle()) + Math.PI /2;
            double ipotenuse = Math.sqrt(x * x + y * y);
            double rotate  = gamepad1.right_stick_x * 0.50;
            double strafe  = Math.sin(direction) * ipotenuse;
            double forward = Math.cos(direction) * ipotenuse;
//
            double lR = - 0.8*robot.SQRT(-strafe - forward - rotate);
            double rF = 0.8*robot.SQRT(strafe + forward - rotate);
            double lF = - 0.8*robot.SQRT(strafe - forward - rotate);
            double rR = 0.8*robot.SQRT(-strafe + forward - rotate);

//            cand esti in shipping hub te misti mai rapid
//            if(pozitie == "shipping_hub"){
//                lR = - 0.8*robot.SQRT(-strafe - forward - rotate);
//                rF = 0.8*robot.SQRT(strafe + forward - rotate);
//                lF = - 0.8*robot.SQRT(strafe - forward - rotate);
//                rR = 0.8*robot.SQRT(-strafe + forward - rotate);
//            }

            //cand esti in warehouse te misti mai incet
            if(pozitie == "warehouse"){
                lR = - 0.3*robot.SQRT(-strafe - forward - rotate);
                rF = 0.3*robot.SQRT(strafe + forward - rotate);
                lF = - 0.3*robot.SQRT(strafe - forward - rotate);
                rR = 0.3*robot.SQRT(-strafe + forward - rotate);
            }

            robot.setMotorPowers(lF,lR,rR,rF);

            //Updatez pozitia robotului
            robot.updatePoseEstimate();

            // Senzori distanta de la senzori culoare
            double dD = robot.distantaDreapta.getDistance(DistanceUnit.CM);
            double dS = robot.distantaStanga.getDistance(DistanceUnit.CM);
            double dC = robot.distantacolor.getDistance(DistanceUnit.CM);

            //senzori culoare
            double rStanga = robot.culoareSpate.red();
            double gStanga = robot.culoareSpate.green();
            double bStanga = robot.culoareSpate.blue();


            //Deschidere/Inchidere Cleste
            if(gamepad1.right_bumper) {
                robot.cleste.setPosition(0.6);
            }
            if(gamepad1.left_bumper){
                robot.cleste.setPosition(0.5);
            }

            //Resetare unghi

            if (gamepad1.x){
                robot.resetAngle();
            }


            //pozitionare robot paralel cu peretele (aliniare teren)
          /*  if(gamepad1.dpad_right){
              // robot.updatePoseEstimate();
                    robot.turn(Math.toRadians(-(robot.getAngle())));
            }*/
            //resetare unghi la peste 360
            if(robot.getAngle()>360 || robot.getAngle() < -360)
                robot.resetAngle();

//            while(gamepad1.dpad_down){
//                while(drf > 50 && drs > 50 ) {
//                    robot.setMotorPowers(0.5, -0.5, 0.5, -0.5);
//                    drf = robot.dreapta_fata.getDistance(DistanceUnit.CM);
//                    drs = robot.dreapta_spate.getDistance(DistanceUnit.CM);
//                }
//                while(drf > 25 && drs > 25 ) {
//                    robot.setMotorPowers(0.4, -0.4, 0.4, -0.4);
//                    drf = robot.dreapta_fata.getDistance(DistanceUnit.CM);
//                    drs = robot.dreapta_spate.getDistance(DistanceUnit.CM);
//                }
//                while(drf > 10 && drs > 10 ) {
//                    robot.setMotorPowers(0.2, -0.2, 0.2, -0.2);
//                    drf = robot.dreapta_fata.getDistance(DistanceUnit.CM);
//                    drs = robot.dreapta_spate.getDistance(DistanceUnit.CM);
//                }
//                while(drf > 5 && drs > 5 ) {
//                    robot.setMotorPowers(0.1, -0.1, 0.1, -0.1);
//                    drf = robot.dreapta_fata.getDistance(DistanceUnit.CM);
//                    drs = robot.dreapta_spate.getDistance(DistanceUnit.CM);
//                }
//                robot.stopDriving();
//            }

            /*if(gamepad1.dpad_left){
                drive.updatePoseEstimate();
                Pose2d currentPose;
                drive.updatePoseEstimate();
                currentPose = drive.getPoseEstimate();
                Trajectory traj1 = drive.trajectoryBuilder(currentPose,true)
                        .splineToConstantHeading(new Vector2d(-20,5),0)
                        .build();

                drive.followTrajectory(traj1);
                drive.updatePoseEstimate();
            }*/

            //detectare culoare alba => ori merge in shipping hub, ori merge in warehouse


            //detectarea culorii albe
            if (robot.culoareSpate.red()>=80 && robot.culoareSpate.green()>=150 && robot.culoareSpate.blue() >= 150 && robot.culoareSpate.alpha()>=90){
                //robot.setPoseEstimate(new Pose2d());
                if(pozitie == "shipping_hub" && time.time() > 1){
                    pozitie = "warehouse";
                    time.reset();
                }
                else if(pozitie == "warehouse" && time.time() > 1) {
                    pozitie = "shipping_hub";
                    time.reset();
                }
                robot.setPoseEstimate(new Pose2d(28.594453496583984,-62.96759694447507));
            }


            /** GAMEPAD 2 */

            //actionare brat
            double bS;
            double bJ;
            bS=gamepad2.right_trigger;
            bJ=gamepad2.left_trigger;
           /* if(bS>0 && ticks <= 1480)
                robot.ridicareBrat.setPower(bS);
            else if(bJ>0 && ticks > 0)
                robot.ridicareBrat.setPower(-bJ);
            else
                robot.ridicareBrat.setPower(0);*/

            //carusel
            if(gamepad2.right_bumper)
                robot.rata.setPower(1);
            if(gamepad2.left_bumper)
                robot.rata.setPower(0);

            //rotatie brat
            while(gamepad2.a) {
                if(robot.PivotBrat.getPosition()<=0.75)
                    robot.PivotBrat.setPosition(robot.PivotBrat.getPosition()+0.02);
            }
            if(gamepad2.x) {
                double p = 0.5;
                while(p<=0.75) {
                    robot.PivotBrat.setPosition(p);
                    sleep(100);
                    p+=0.02;
                }
            }

            ///nivele brat
            /*
            nivel 1 = 380 ticks
            nivel 2 = 860 ticks
            nivel 3 = 1280 ticks
             */
            if(gamepad2.dpad_up) {
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.ridicareBrat.setTargetPosition(1280);
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (robot.ridicareBrat.isBusy()) {
                    robot.ridicareBrat.setPower(1);
                }
            }
            if(gamepad2.dpad_down){
                    robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.ridicareBrat.setTargetPosition(0);
                    robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while(robot.ridicareBrat.isBusy() ) {
                        robot.ridicareBrat.setPower(1);
                    }
                }



//            //actionare brat care nu merge
//            robot.ridicareBrat.setPower(-gamepad2.right_trigger);
//            robot.ridicareBrat.setPower(gamepad2.left_trigger);

//          telemetry.addData("UNGHI: ",(robot.getAngle()));
//          telemetry.update();
            dashboardTelemetry.addData("Unghi", (robot.getAngle()));
//            dashboardTelemetry.addData("dreapta fata", drf);
//            dashboardTelemetry.addData("dreapta spate", drs);
//            dashboardTelemetry.addData("stanga fata", stf);
//            dashboardTelemetry.addData("stanga spate", sts);
            dashboardTelemetry.addData("ticks:", ticks);
//            telemetry.addData("Color","R %d G %d B %d", robot.culoareFata.red(), robot.culoareFata.blue(), robot.culoareFata.green());
            dashboardTelemetry.addData("Color","R %d G %d B %d", robot.culoareSpate.red(), robot.culoareSpate.blue(), robot.culoareSpate.green());
           dashboardTelemetry.addData("Color", robot.culoareSpate.alpha());
//            dashboardTelemetry.addData("distatantacolor", dC);
            dashboardTelemetry.addData("x", robot.getPoseEstimate().getX());
            dashboardTelemetry.addData("y", robot.getPoseEstimate().getY());
            dashboardTelemetry.addData("Pozitie robot",pozitie);
            dashboardTelemetry.update();
        }
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ridicareBrat.setTargetPosition(0);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(robot.ridicareBrat.isBusy() ) {
            robot.ridicareBrat.setPower(0);
        }


    }
}