package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LimeLightBallDetection", group = "Vision")
public class  LimeLightBallDetection extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize the Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "LimeLightBallDetection");

        // Switch to the correct pipeline
        //pipeline 1 is for green
        //pipeline 2 is for purple
        limelight.pipelineSwitch(1);
        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            //getting latest results
            limelight.pipelineSwitch(1); //green
            LLResult result = limelight.getLatestResult();

            if (result != null) {
                if (result.isValid()) {
                    //TARGETING INFORMATION tx, ty, ta
                    double tx = result.getTx(); // Horizontal offset
                    double ty = result.getTy(); // Vertical offset
                    double ta = result.getTa(); // Target area (size)

                    telemetry.addLine("Green Ball Detected!");
                    telemetry.addData("tx", tx);
                    telemetry.addData("ty", ty);
                    telemetry.addData("ta", ta);
                    telemetry.update();
                }
            }

            limelight.pipelineSwitch(3); //purple
            result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    //TARGETING INFORMATION tx, ty, ta
                    double tx = result.getTx(); // Horizontal offset
                    double ty = result.getTy(); // Vertical offset
                    double ta = result.getTa(); // Target area (size)

                    telemetry.addLine("Purple Ball Detected!");
                    telemetry.addData("tx", tx);
                    telemetry.addData("ty", ty);
                    telemetry.addData("ta", ta);
                    telemetry.update();
                }
            }
        }

        limelight.stop(); // Stop the Limelight when opmode ends
    }
}