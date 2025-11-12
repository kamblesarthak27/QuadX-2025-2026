package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp Decode12", group="TeleOp")
public class BrokenTeleOp extends OpMode {

    private DcMotorEx outtake, outtake2, fl, fr, bl, br, frontIntake, backIntake;
    private Servo shroud;

    // PID constants
    private  double Kp = 0.2;
    private static final double Ki = 0.0;
    private static final double Kd = 0.0;
    private static final double Kf = 0.0;
    private  double TARGET_VELOCITY = 1000;
    private static final double VELOCITY_TOLERANCE = 50;
    private static final double INTEGRAL_CLAMP = 0.2;
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double SLOW_MODE_MULTIPLIER = 0.5;
    private static final double PRECISION_MODE_MULTIPLIER = 0.25;

    private double integralSum = 0;
    private double previousError = 0;
    boolean outtakeActive = false; // Tracks if the motors should currently be running
    boolean triggerWasPressed = false; // Tracks the button state from the last loop
    private long lastLoopTime;
    double pos = 0;
    boolean dpadRightWasPressed = false;
    boolean dpadLeftWasPressed = false;
    boolean dpadUpWasPressed = false;
    boolean dpadDownWasPressed = false;


    @Override
    public void init() {
        // Initialize outtake motors
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorEx.Direction.REVERSE);
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shroud = hardwareMap.get(Servo.class, "shroudCont");
        // Initialize intake motors
        frontIntake = hardwareMap.get(DcMotorEx.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotorEx.class, "backIntake");
        frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        backIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize drivetrain motors
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastLoopTime = System.nanoTime();
    }

    @Override
    public void init_loop() {
        outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Calculate delta time in seconds
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastLoopTime) / 1e9;
        lastLoopTime = currentTime;

        // --- Outtake PID Control ---
        boolean triggerPressed = gamepad2.right_trigger > TRIGGER_THRESHOLD;
        if (gamepad1.a && !dpadRightWasPressed) {
            Kp += 0.1;
            pos = Range.clip(pos, 0.48, 1);
            //shroud.setPosition(pos);
        }
// Update the 'was pressed' variable at the END of the section
        dpadRightWasPressed = gamepad1.a;

// --- Left D-pad Logic (Corrected) ---
        if (gamepad1.b && !dpadLeftWasPressed) {
            Kp -= 0.1;
            pos = Range.clip(pos, 0.48, 1);
            //shroud.setPosition(pos);
        }
// Update the 'was pressed' variable at the END of the section
        dpadLeftWasPressed = gamepad1.b;

// --- Up D-pad Logic (Corrected) ---
        if (gamepad2.dpad_up && !dpadUpWasPressed){
            TARGET_VELOCITY += 50;
        }
// Update the 'was pressed' variable at the END of the section
        dpadUpWasPressed = gamepad2.dpad_up;


// --- Down D-pad Logic (Corrected) ---
// Note: Your original down logic was missing the !dpadDownWasPressed check,
// so this also fixes that.
        if (gamepad2.dpad_down && !dpadDownWasPressed){
            TARGET_VELOCITY -= 50;
        }
// Update the 'was pressed' variable at the END of the section
        dpadDownWasPressed = gamepad2.dpad_down;

        boolean currentTriggerState = gamepad1.right_bumper; // Replace with your actual trigger/button variable

// Check for the RISING EDGE of the trigger button
        if (currentTriggerState && !triggerWasPressed) {
            // A single press occurred, toggle the state
            outtakeActive = !outtakeActive;

            // If we just turned it ON, reset PID values for a clean start
            if (outtakeActive) {
                integralSum = 0;
                previousError = 0;
            }
        }
// Update the 'was pressed' variable at the end of the check
        triggerWasPressed = currentTriggerState;
// --- End Toggle Logic ---


        if(gamepad1.right_bumper){
            outtake.setPower(Kp);
            outtake2.setPower(Kp);
        } else if (gamepad1.left_bumper){
            outtake.setPower(0);
            outtake2.setPower(0);
        }


// --- Motor Control and Telemetry Logic (Modified) ---
        if (outtakeActive) {
            /*
            double currentVelocity = outtake.getVelocity();
            double error = TARGET_VELOCITY - currentVelocity;

            // PID calculations
            integralSum += error * dt;
            integralSum = Range.clip(integralSum, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
            double derivative = (dt > 0) ? (error - previousError) / dt : 0;
            double motorPower = Kp * error + Ki * integralSum + Kd * derivative + Kf * TARGET_VELOCITY;
            motorPower = Range.clip(motorPower, -1.0, 1.0);
            */
            //outtake.setPower(Kp);
            //outtake2.setPower(Kp);

            //previousError = error;

            // Telemetry (only when active)
            telemetry.addData("Outtake Status", "ACTIVE");
            //telemetry.addData("Current Velocity", currentVelocity);
            //telemetry.addData("Error", error);
            telemetry.addData("Kp Value", Kp);
        } else {
            // If we are not active, ensure motors are off
            //outtake.setPower(0);
            //outtake2.setPower(0);

            // Telemetry (when inactive)
            telemetry.addData("Outtake Status", "INACTIVE");
            //telemetry.addData("Target Velocity", TARGET_VELOCITY);
            telemetry.addData("Kp Value", Kp);
        }

        // --- Drivetrain Control ---
        double drive = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        double flPower = drive + turn - strafe;
        double frPower = drive - turn + strafe;
        double blPower = drive + turn + strafe;
        double brPower = drive - turn - strafe;

        // Apply speed modifiers
        double speedMultiplier = 1.0;
        if (gamepad1.right_bumper) {
            speedMultiplier = SLOW_MODE_MULTIPLIER;
        }
        if (gamepad1.left_bumper) {
            speedMultiplier = PRECISION_MODE_MULTIPLIER;
        }

        if (speedMultiplier != 1.0) {
            flPower *= speedMultiplier;
            frPower *= speedMultiplier;
            blPower *= speedMultiplier;
            brPower *= speedMultiplier;
        }

        // Clip and set drivetrain powers
        fl.setPower(Range.clip(flPower, -1.0, 1.0));
        fr.setPower(Range.clip(frPower, -1.0, 1.0));
        bl.setPower(Range.clip(blPower, -1.0, 1.0));
        br.setPower(Range.clip(brPower, -1.0, 1.0));

        // --- Intake Control ---
        if (gamepad2.right_bumper) {
            frontIntake.setPower(1);
            backIntake.setPower(1);
        } else if (gamepad2.left_bumper) {
            frontIntake.setPower(-1);
            backIntake.setPower(-1);
        } else if (gamepad2.a) {
            frontIntake.setPower(1);
        }else if (gamepad2.b) {
            backIntake.setPower(1);
        } else if (gamepad2.x) {
            frontIntake.setPower(-1);
        }else if (gamepad2.y) {
            backIntake.setPower(-1);
        } else {
            frontIntake.setPower(0);
            backIntake.setPower(0);
        }

        telemetry.update();
    }
}