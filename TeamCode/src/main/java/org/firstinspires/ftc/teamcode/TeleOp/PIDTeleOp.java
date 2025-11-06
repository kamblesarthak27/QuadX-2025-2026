package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    private ElapsedTime time = new ElapsedTime();
    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx bl;
    private DcMotorEx br;
    private Servo servo;
    private DistanceSensor dist;
    private ColorSensor color;
    private DcMotorEx outtake;
    private DcMotorEx outtake2;
    private final double TARGET_VELOCITY = 1650; //  TUNE THIS
    private final double VELOCITY_TOLERANCE = 50; // Ticks/sec tolerance for the PID

    private double Kp = 0.005; // Proportional constant
    private double Ki = 0.0;    // Integral constant
    private double Kd = 0.0;    // Derivative constant
    private double Kf = 0.0;    // Feedforward constant (helps with constant speed)

    private double slowdown = 0;

    private double integralSum = 0;
    private double previousError = 0;
    @Override
    public void init() {
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

        dist = hardwareMap.get(DistanceSensor.class, "dist");
        color = hardwareMap.get(ColorSensor.class, "color");

        servo = hardwareMap.servo.get("Servo");
    }

    @Override
    public void loop() {
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

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

        double distance = dist.getDistance(DistanceUnit.CM);

        dist.getDistance(DistanceUnit.CM);
        dist.getDistance(DistanceUnit.MM);
        dist.getDistance(DistanceUnit.INCH);
        dist.getDistance(DistanceUnit.METER);

        telemetry.addData("Distance", distance);

        double currentVelocity = outtake.getVelocity();
        double targetVelocity = 0;

        // Check if the right trigger is pressed
        if (gamepad1.right_trigger > 0.1) {
            targetVelocity = TARGET_VELOCITY;
            slowdown = 1;
        } else {slowdown = 0;}

        // --- PID Control Calculations ---
        double error = targetVelocity - currentVelocity;

        // Proportional term
        double p = Kp * error * slowdown;

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
        double motorPower = p + i + d + f;
        motorPower = Range.clip(motorPower, -1.0, 1.0);

        outtake.setPower(gamepad1.right_trigger);
        outtake2.setPower(gamepad1.right_trigger);

        // Reset the time for the next loop iteration
        time.reset();
        previousError = error;

        int red = color.red();
        int green = color.green();
        int blue = color.blue();

        int[] RGBColor = {red, green, blue};

        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);
    }

}