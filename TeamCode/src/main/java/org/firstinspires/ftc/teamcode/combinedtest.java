package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Combined Test", group="Test")
public class combinedtest extends OpMode {

    DcMotorEx testmotor;
    Servo hoodservo;

    double shooter_value = 0.0;
    double servo_value = 0.5;

    boolean xAlrPressed = false;
    boolean yAlrPressed = false;

    ElapsedTime motorDebounce = new ElapsedTime();
    ElapsedTime servoDebounce = new ElapsedTime();

    @Override
    public void init() {
        testmotor = hardwareMap.get(DcMotorEx.class, "testemotor");
        testmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        testmotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // IMPORTANT
        testmotor.setDirection(DcMotorEx.Direction.REVERSE);

        hoodservo = hardwareMap.get(Servo.class, "hood");
        hoodservo.setPosition(servo_value);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // MOTOR CONTROL (X / Y)
        if (motorDebounce.milliseconds() > 300) {

            if (gamepad1.x && !xAlrPressed) {
                shooter_value += 0.1;
                motorDebounce.reset();
            }

            if (gamepad1.y && !yAlrPressed) {
                shooter_value -= 0.1;
                motorDebounce.reset();
            }
        }

        xAlrPressed = gamepad1.x;
        yAlrPressed = gamepad1.y;


        if (gamepad1.right_bumper) {
            shooter_value = 0.0;
            servo_value = 0.5;
        }

        testmotor.setPower(shooter_value);

        // SERVO CONTROL (A / B)
        if (servoDebounce.milliseconds() > 150) {
            if (gamepad1.a) {
                servo_value += 0.05;
                servoDebounce.reset();
            } else if (gamepad1.b) {
                servo_value -= 0.05;
                servoDebounce.reset();
            }
        }

        servo_value = Math.max(0.0, Math.min(servo_value, 1.0));
        hoodservo.setPosition(servo_value);

        telemetry.addData("Motor Power", shooter_value);
        telemetry.addData("Hood Position", servo_value);
        telemetry.update();
    }
}
