package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "LimeLightABC", group = "Vision")
public class LimeLightABC extends LinearOpMode {

    private Limelight3A limelight;
    private static final int LEFT_TAG = 1;
    private static final int CENTER_TAG = 2;
    private static final int RIGHT_TAG = 3;

    @Override
    public void runOpMode() {
        // Keep hardware mapping
        limelight = hardwareMap.get(Limelight3A.class, "LimeLightABC");

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Ready - Point at AprilTags");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }
            }
            telemetry.update();
        }
    }
}

class LimelightHelper {
    private final String cameraName;
    private final WebcamName limelightCamera;

    public LimelightHelper(HardwareMap hardwareMap, String cameraName) {
        this.cameraName = cameraName;
        this.limelightCamera = hardwareMap.get(WebcamName.class, cameraName);
    }

    // These simulate or later can connect to Limelight SDK calls
    public boolean hasTarget() {
        // Replace this with actual Limelight SDK call later
        return Math.random() > 0.3; // Simulated detection
    }

    public double getTargetX() {
        // Simulate horizontal offset
        return (Math.random() * 30) - 15;
    }

    public double getTargetY() {
        // Simulate vertical offset
        return (Math.random() * 20) - 10;
    }

    public double getTargetArea() {
        // Simulate target area percent
        return Math.random();
    }

    public void setPipeline(int pipeline) {
        // Placeholder for setting pipeline
    }

    public void setLedMode(int mode) {
        // Placeholder for setting LED mode
    }
}