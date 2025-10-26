package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class ServoPractice extends LinearOpMode {
    private Servo servo;
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "axon");

        waitForStart();

        servo.setPosition(0);
        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo.setPosition(0); // Move to position 0
            } else if (gamepad1.b) {
                servo.setPosition(1); // Move to position 1
            }

            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}
