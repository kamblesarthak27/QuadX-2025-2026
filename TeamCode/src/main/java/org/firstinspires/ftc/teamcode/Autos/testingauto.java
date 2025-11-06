package org.firstinspires.ftc.teamcode.Autos;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "testingAuto", group = "Autonomous")
public class testingauto extends LinearOpMode {

    public class Intake {
        private DcMotorEx intakeFront;
        private DcMotorEx intakeBack;

        public Intake(HardwareMap hardwareMap) {
            intakeFront = hardwareMap.get(DcMotorEx.class, "frontIntake");
            intakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeBack = hardwareMap.get(DcMotorEx.class, "backIntake");
            intakeBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class RunIntakeForTime implements Action {
            private final double durationSeconds;
            private final double powerFront;
            private final double powerBack;
            private ElapsedTime runtime = new ElapsedTime();
            private boolean initialized = false;

            public RunIntakeForTime(double powerFront, double powerBack, double durationSeconds) {
                this.powerFront = powerFront;
                this.powerBack = powerBack;
                this.durationSeconds = durationSeconds;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    runtime.reset(); // Start the timer
                    intakeFront.setPower(powerFront);
                    intakeBack.setPower(powerBack);
                    initialized = true;
                }

                // Check if the elapsed time is less than the desired time
                if (runtime.seconds() < durationSeconds) {
                    return true; // Continue running
                } else {
                    intakeFront.setPower(0); // Stop the motors
                    intakeBack.setPower(0);
                    return false; // Stop running
                }
            }
        }
        public Action runForDuration(double powerFront, double powerBack, double durationSeconds) {
            return new RunIntakeForTime(powerFront, powerBack, durationSeconds);
        }
    }

    public class Outtake {
        private DcMotorEx outtake1;
        private DcMotorEx outtake2;

        public Outtake (HardwareMap hardwareMap){
            outtake1 = hardwareMap.get(DcMotorEx.class, "outtake");
            outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
            outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
            outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class RunOuttakeForTime implements Action {
            private final double durationSeconds;
            private final double power;
            private ElapsedTime runtime = new ElapsedTime();
            private boolean initialized = false;

            public RunOuttakeForTime(double power, double durationSeconds) {
                this.power = power;
                this.durationSeconds = durationSeconds;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    runtime.reset(); // Start the timer
                    outtake1.setPower(power);
                    outtake2.setPower(power);
                    initialized = true;
                }

                // Check if the elapsed time is less than the desired time
                if (runtime.seconds() < durationSeconds) {
                    return true; // Continue running
                } else {
                    outtake1.setPower(0); // Stop the motors
                    outtake2.setPower(0);
                    return false; // Stop running
                }
            }
        }
        public Action runForDuration(double power, double durationSeconds) {
            return new Outtake.RunOuttakeForTime(power, durationSeconds);
        }
    }


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(60, -10, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Outtake shooter = new Outtake(hardwareMap);

        // vision here that outputs pattern (Do later)
        // int visionOutputPosition = 1;

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(40, -60), Math.toRadians(270))
                .waitSeconds(3);
        /*
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
                */
        Action trajectoryActionCloseOut = traj1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(60, -10), Math.toRadians(195))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            //int position = visionOutputPosition;
            //telemetry.addData("Position during Init", position);
            //telemetry.update();
        }

        //int startPosition = visionOutputPosition;
        //telemetry.addData("Starting Position", startPosition);
        //telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        /*
        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }
        */

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                traj1.build(),
                                intake.runForDuration(1,-1,5)
                        ),
                        trajectoryActionCloseOut
                )
        );
    }
}
