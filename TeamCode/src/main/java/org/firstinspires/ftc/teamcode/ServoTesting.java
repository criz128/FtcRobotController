package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp

public class ServoTesting extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor test_motor;
    private Servo test_servo;
    private TouchSensor test_touch;


    @Override
    public void runOpMode() {
        test_servo = hardwareMap.get(Servo.class, "axon");

        test_servo.setPosition(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.y) {
                //move to position 0
                test_servo.setPosition(0);

            } else if (gamepad1.x || gamepad1.b) {
                //move to position 0.5
                test_servo.setPosition(0.5);

            } else if (gamepad1.a) {
                //move to position 1
                test_servo.setPosition(1);
            }
            telemetry.addData("Servo Position", test_servo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}