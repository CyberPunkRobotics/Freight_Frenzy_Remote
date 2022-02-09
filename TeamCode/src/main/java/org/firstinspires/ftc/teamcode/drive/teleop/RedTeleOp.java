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


@TeleOp(name = "RedTeleOp", group = "MecanumBot")
public class RedTeleOp extends LinearOpMode {

    private SampleMecanumDrive robot = null;
    private final double DELAY = 0.2;
    public ElapsedTime time = new ElapsedTime();
    public ElapsedTime brat = new ElapsedTime();

    double power=0, lowPower = 0.3, highPower = 0.8;

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

        //contor tickuri ridicare/coborare brat
        int ticks;

        //pt determinare pozitie robot (la inceput e inafara warehouse-ului)
        //warehouse = false , sh = true
        boolean pozitie = true;

        String cfbrat = "default";

        //telemetry pe dashboard
        Telemetry dashboardTelemetry;
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //pozitiile inititale ale bratului + clestelui +cap
        //robot.intake.setPosition(0.23);
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

            double direction = Math.atan2(x, y) - Math.toRadians(robot.getAngle()) + Math.PI / 2;
            double ipotenuse = Math.sqrt(x * x + y * y);
            double rotate = gamepad1.right_stick_x * 0.50;
            double strafe = Math.sin(direction) * ipotenuse;
            double forward = Math.cos(direction) * ipotenuse;

            double lR = -power * robot.SQRT(-strafe - forward - rotate);
            double rF = power * robot.SQRT(strafe + forward - rotate);
            double lF = -power * robot.SQRT(strafe - forward - rotate);
            double rR = power * robot.SQRT(-strafe + forward - rotate);

            robot.setMotorPowers(lF, lR, rR, rF);

//            //cand esti in warehouse te misti mai incet
//            if (!pozitie) {
//                power = lowPower;
////                lR = -power * robot.SQRT(-strafe - forward - rotate);
////                rF = power * robot.SQRT(strafe + forward - rotate);
////                lF = -power * robot.SQRT(strafe - forward - rotate);
////                rR = power * robot.SQRT(-strafe + forward - rotate);
//            }
//            else {
//                power = highPower;
//            }



            //Senzori distanta de la senzori culoare
            double dD = robot.distantaDreapta.getDistance(DistanceUnit.CM);
            double dS = robot.distantaStanga.getDistance(DistanceUnit.CM);
            double dI = robot.distantaIntake.getDistance(DistanceUnit.CM);

            //senzori culoare
            double rStanga = robot.culoareSpate.red();
            double gStanga = robot.culoareSpate.green();
            double bStanga = robot.culoareSpate.blue();

            //Deschidere/Inchidere Cleste
            /*if (gamepad1.right_bumper) {
                robot.intake.setPosition(0.32);
            }
            if (gamepad1.left_bumper) {
                robot.intake.setPosition(0.23);
            }*/

            //actionare intake
            if(gamepad1.right_bumper && dI > 1.9)
                robot.intake.setPower(0.9);
            else if(gamepad1.left_bumper)
                robot.intake.setPower(-0.9);
            else robot.intake.setPower(0);

//            //intake automat
//            if(dI <= 4.3 && aruncare.time() > 1 && dI > 1.9)
//                robot.intake.setPower(0.9);
//            if(dI > 4.3)
//                aruncare.reset();

            //Resetare unghi
            if (gamepad1.square) {
                robot.resetAngle();
            }

            //pozitionare robot paralel cu peretele (aliniare teren)
          /*  if(gamepad1.dpad_right){
              // robot.updatePoseEstimate();
                    robot.turn(Math.toRadians(-(robot.getAngle())));
            }*/

            //resetare unghi la peste 360
            if (robot.getAngle() > 360 || robot.getAngle() < -360)
                robot.resetAngle();

            /// alb v mica 119 184 210
            ///rosu v mica 195 155 196
            /// alb v mare 100-150 , 150, 160-200
            /// rosu v mare 100,160, 180
            //detectarea culorii albe
////            daca detecteaza culoarea alba => ai trecut peste linia dintre warehouse si shipping hub
////            if (robot.culoareSpate.red() >= 100 && robot.culoareSpate.green() >= 150 && robot.culoareSpate.blue() >= 150 && robot.culoareSpate.alpha() >= 90) {
//            if (robot.culoareSpate.getNormalizedColors().toColor() > 1850000000) {
//                if (!pozitie && time.time() > 1.512) {
//                    pozitie = true;
//                    time.reset();
////                } else if (pozitie && time.time() > 1.512) {
////                    pozitie = false;
////                    time.reset();
////                }
//                //pentru a nu pierde din pozitia robotului, ne folosim de coordonatele
//                //liniei albe si resetam pozitia robotului pentru a stii unde se afla
//                robot.setPoseEstimate(new Pose2d(28.594453496583984, -62.96759694447507));
//            }
//            if (robot.culoareFata.getNormalizedColors().toColor() > 1250000000) {
//                if (!pozitie && time.time() > 1.512) {
//                    pozitie = true;
//                    time.reset();
//                } else if (pozitie && time.time() > 1.512) {
//                    pozitie = false;
//                    time.reset();
//                }
//                //pentru a nu pierde din pozitia robotului, ne folosim de coordonatele
//                //liniei albe si resetam pozitia robotului pentru a stii unde se afla
//                robot.setPoseEstimate(new Pose2d(28.594453496583984, -62.96759694447507));
//            }



            /** GAMEPAD 2 */

            //actionare brat
            double bS;
            double bJ;
            bS = gamepad2.right_trigger;
            bJ = gamepad2.left_trigger;

            //ridicare/coborare brat
            if(bS > 0)
                robot.ridicareBrat.setPower(bS);
            else if(bJ > 0)
                robot.ridicareBrat.setPower(-bJ);
            else robot.ridicareBrat.setPower(0);


            //carusel
            if(gamepad2.left_bumper)
                robot.rata.setPower(1);
            else
                robot.rata.setPower(0);

            //reducere viteza
       //     if(gamepad1.right_stick_button ) {
//                if (!pozitie)
//                    pozitie = true;
//                else if (pozitie)
//                    pozitie = false;
//                time.reset();
//            }


            //rotatie brat
            if(gamepad2.dpad_left) {
                if(robot.PivotBrat.getPosition()<=0.75)
                    robot.PivotBrat.setPosition(robot.PivotBrat.getPosition()+0.02);
            }
            if(gamepad2.dpad_right){
                if(robot.PivotBrat.getPosition()>=0.50)
                    robot.PivotBrat.setPosition(robot.PivotBrat.getPosition()-0.02);
            }

            //capping
            if(gamepad2.square)
                robot.cap.setPosition(0); //brat jos
            if(gamepad2.circle)
                robot.cap.setPosition(0.6);

            if(gamepad2.dpad_down){
                cfbrat = "stanga";
                    if(brat.seconds() == 0.5)
                        robot.PivotBrat.setPosition(0.02);
                    brat.reset();
                    if(robot.PivotBrat.getPosition() >= 0.75)
                        cfbrat = "oprit";
                brat.reset();
            }



            /*if(gamepad2.dpad_right)
            if(gamepad2.x) {
                double p = 0.5;
                while(p<=0.75) {
                    robot.PivotBrat.setPosition(p);
                    sleep(100);
                    p+=0.02;
                }
            }*/

            ///nivele brat
            /*
            nivel 1 = 380 ticks
            nivel 2 = 860 ticks
            nivel 3 = 1280 ticks
             */
            /*if(gamepad2.dpad_up) {
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.ridicareBrat.setTargetPosition(1280);
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                nivel_brat = "sus";
            }
                if(nivel_brat == "sus" && robot.ridicareBrat.getCurrentPosition()<=1280) {
                    robot.ridicareBrat.setPower(1);
                }

            if(gamepad2.dpad_down){
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.ridicareBrat.setTargetPosition(0);
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    nivel_brat = "jos";
            }
                if(nivel_brat == "jos" && robot.ridicareBrat.getCurrentPosition()>=0) {
                    robot.ridicareBrat.setPower(1);
                }

                if(robot.ridicareBrat.getCurrentPosition() == 1280 || robot.ridicareBrat.getCurrentPosition() == 0)
                    nivel_brat = "-";*/



//          telemetry.addData("UNGHI: ",(robot.getAngle()));
//          telemetry.update();
//            dashboardTelemetry.addData("Unghi", (robot.getAngle()));
//            dashboardTelemetry.addData("dreapta fata", drf);
//            dashboardTelemetry.addData("dreapta spate", drs);
//            dashboardTelemetry.addData("stanga fata", stf);
//            dashboardTelemetry.addData("stanga spate", sts);
            dashboardTelemetry.addData("ticks:", ticks);
//            telemetry.addData("Color","R %d G %d B %d", robot.culoareFata.red(), robot.culoareFata.blue(), robot.culoareFata.green());
            dashboardTelemetry.addData("Color fata ", robot.culoareIntake.getNormalizedColors().toColor());
            dashboardTelemetry.addData("Color spate ", robot.culoareSpate.getNormalizedColors().toColor());
            dashboardTelemetry.addData("Color","R %d G %d B %d", robot.culoareSpate.red(), robot.culoareSpate.blue(), robot.culoareSpate.green());
//           dashboardTelemetry.addData("Color", robot.culoareSpate.alpha());
//            dashboardTelemetry.addData("distatantacolor", dC);
            dashboardTelemetry.addData("x", robot.getPoseEstimate().getX());
            dashboardTelemetry.addData("y", robot.getPoseEstimate().getY());
            dashboardTelemetry.addData("Pozitie robot",pozitie);
            dashboardTelemetry.addData("disttanta", robot.distantaIntake.getDistance(DistanceUnit.CM));
            dashboardTelemetry.update();
        }

    }
}