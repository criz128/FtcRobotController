package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

@TeleOp(name="Brushland testing")
public class ColorSensorNServo extends OpMode{
    DigitalChannel brushlandColorIntakePin0;
    DigitalChannel brushlandColorIntakePin1;
    ColorRangefinder crf;


    @Override
    public void init() {
        brushlandColorIntakePin0 = hardwareMap.get(DigitalChannel.class, "digital_0");
        brushlandColorIntakePin0.setMode(DigitalChannel.Mode.INPUT);// by default
        brushlandColorIntakePin1 = hardwareMap.get(DigitalChannel.class, "digital_1");
        brushlandColorIntakePin1.setMode(DigitalChannel.Mode.INPUT);// by default

        crf = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "Color"));


        // Set purple range
        crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 240 / 360.0 * 255, 300 / 360.0 * 255); // purple
        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 60 / 360.0 * 255, 180 / 360.0 * 255); // green
        // Set green range
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean pin0 = brushlandColorIntakePin0.getState();
        boolean pin1 = brushlandColorIntakePin1.getState();

        telemetry.addData("digital 0", pin0);
        telemetry.addData("digital 1", pin1);
        telemetry.update();

        boolean purpleDetected = brushlandColorIntakePin0.getState();
        boolean greenDetected = brushlandColorIntakePin1.getState();

        telemetry.addData("Purple detected", purpleDetected);
        telemetry.addData("Green detected", greenDetected);
        telemetry.update();

    }
}