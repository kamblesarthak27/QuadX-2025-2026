package org.firstinspires.ftc.teamcode.PIDs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Outtake {
    private ElapsedTime time;
    private DcMotorEx outtake;
    private double tolerance;
    double previousTime;
    double currentTime;
    double previousError;
    double error;
    double Kp;
    double Ki;
    double Kd;
    double max_i;
    double min_i;
    double motorPower;

    public void init() {
        time = new ElapsedTime();

        outtake = hardwareMap.get(DcMotorEx.class, "liftLeft");
        outtake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tolerance = 0.05;
        previousTime = 0;
        previousError = 0;
        Kp = 0;
        Ki = 0;
        Kd = 0;
        max_i = 0.03;
        min_i = -0.03;
        motorPower = 0;
        currentTime = 0;
        error = 0;
    }

    public void outtake(double targetPower) {
        init();
        double p = 0;
        double i = 0;
        double d = 0;

        double currentPower = (double)outtake.getPower();

        while (Math.abs(targetPower - currentPower) > tolerance) {
            currentTime = time.milliseconds();
            error = targetPower - currentPower;
            if (error < 0) {
                p = Kp * error;
                i += (Ki * (error * (currentTime - previousTime)));
                i = Range.clip(i, min_i, max_i);
                d = Kd * (error - previousError) / (currentTime - previousTime);
                motorPower = p + i + d;
            } else {
                motorPower = 0;
            }

            previousError = error;
            previousTime = currentTime;
            currentPower = (double)outtake.getPower();
            telemetry.addData("Outtake Power: ", outtake.getPower());
            telemetry.update();
            outtake.setPower(motorPower);
        }
    }
}