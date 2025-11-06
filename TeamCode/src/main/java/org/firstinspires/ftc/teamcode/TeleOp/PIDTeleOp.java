package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp Shooter", group="TeleOp")
public class TeleOpshooter extends OpMode {

    private ElapsedTime time = new ElapsedTime();

    // Use this for the sorter if needed
    // IntakeSorter sorter = new IntakeSorter();
    private DcMotorEx outtake;
    private DcMotorEx outtake2;// Use DcMotorEx for velocity control
    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx bl;
    private DcMotorEx br;
    private DcMotorEx frontIntake;
    private DcMotorEx backIntake;

    // PID constants. These values need to be tuned for your robot.
    private double Kp = 0.005; // Proportional constant
    private double Ki = 0.0;    // Integral constant
    private double Kd = 0.0;    // Derivative constant
    private double Kf = 0.0;    // Feedforward constant (helps with constant speed)

    private double slowdown = 0;

    private double integralSum = 0;
    private double previousError = 0;

    private final double TARGET_VELOCITY = 6000; //  TUNE THIS
    private final double VELOCITY_TOLERANCE = 50; // Ticks/sec tolerance for the PID

    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorEx.Direction.REVERSE);
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        frontIntake = hardwareMap.get(DcMotorEx.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotorEx.class, "backIntake");

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        backIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void init_loop() {
        // Reset the encoder mode here in case it was changed
        outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double currentVelocity = outtake.getVelocity();
        double targetVelocity = 0;

        // Check if the right trigger is pressed
        if (gamepad2.right_trigger > 0.1) {
            targetVelocity = TARGET_VELOCITY;
            slowdown = 1;
        } else {slowdown = 0;}

        // --- PID Control Calculations ---
        double error = targetVelocity - currentVelocity;

        // Proportional term
        double p = Kp * error;

        // Integral term (used to eliminate steady-state error)
        integralSum += error * time.seconds();
        // Clamp the integral sum to prevent windup
        if (integralSum > 0.2) {
            integralSum = 0.2;
        }
        if (integralSum < -0.2) {
            integralSum = -0.2;
        }
        double i = Ki * integralSum;

        // Derivative term (dampens oscillations)
        double d = Kd * (error - previousError) / time.seconds();

        // Feedforward term (to get motor close to target velocity)
        double f = Kf * targetVelocity;

        // Combine terms for final motor power
        double motorPower = (p + i + d + f) * slowdown;
        motorPower = Range.clip(motorPower, -1.0, 1.0);

        outtake.setPower(motorPower);
        outtake2.setPower(motorPower);

        float drive = gamepad1.left_stick_y;
        float turn = -gamepad1.right_stick_x;
        float strafe = gamepad1.left_stick_x;

        double flPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double frPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double blPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double brPower = Range.clip(drive - turn - strafe, -1.0, 1.0);


        if (gamepad1.right_trigger > 0.0) {
            flPower *= 0.5;
            frPower *= 0.5;
            blPower *= 0.5;
            brPower *= 0.5;
        }

        if (gamepad1.left_bumper) {
            flPower *= 0.25;
            frPower *= 0.25;
            blPower *= 0.25;
            brPower *= 0.25;
        }

        if (gamepad2.right_bumper){
            frontIntake.setPower(1);
            backIntake.setPower(1);
        }
        else if (gamepad2.left_bumper){
            frontIntake.setPower(-1);
            backIntake.setPower(-1);
        } else{
            frontIntake.setPower(0);
            backIntake.setPower(0);
        }

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

        time.reset();
        previousError = error;

        // --- Telemetry for tuning and debugging ---
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("Motor Power", outtake.getPower());
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.update();
    }
}
