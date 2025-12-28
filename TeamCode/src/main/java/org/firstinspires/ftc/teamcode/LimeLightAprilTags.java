package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Turret Auto-Aim", group="Test")
public class LimeLightAprilTags extends OpMode {

    private Limelight3A limelight;
    private Servo yawServo;

    // Start centered
    private double yawPos = 0.5;

    // Tuned for Axon MAX positional servos
    private static final double kYaw = 0.015;

    private static final double DEADZONE = 0.5;      // degrees
    private static final double MAX_STEP = 0.02;     // per loop

    @Override
    public void init() {
        yawServo = hardwareMap.get(Servo.class, "yawServo");
        yawServo.setPosition(yawPos);

        // ----- LIMELIGHT (UNCHANGED) -----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            // ----- LIMELIGHT DATA (UNCHANGED) -----
            double tx = result.getTx(); // horizontal offset (deg)
            double ty = result.getTy(); // vertical offset (deg)

            // ----- YAW SERVO FIX -----
            if (Math.abs(tx) > DEADZONE) {
                double yawAdjust = -tx * kYaw;   // flip sign if needed
                yawAdjust = clip(yawAdjust, -MAX_STEP, MAX_STEP);
                yawPos = clip(yawPos + yawAdjust, 0.0, 1.0);
                yawServo.setPosition(yawPos);
            }



            telemetry.addData("Tx", tx);
            telemetry.addData("Ty", ty);
            telemetry.addData("Yaw Pos", yawPos);

        } else {
            telemetry.addLine("No valid AprilTag");
        }

        telemetry.update();
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
