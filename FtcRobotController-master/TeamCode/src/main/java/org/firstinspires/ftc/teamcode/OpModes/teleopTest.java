package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "testSensor")

public class teleopTest extends LinearOpMode {


    private DistanceSensor clawDistance;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        clawDistance = hardwareMap.get(DistanceSensor.class, "clawDistance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            // generic DistanceSensor methods.

            telemetry.addData("deviceName", clawDistance.getDeviceName());
            telemetry.addData("range", String.format("%.01f mm", clawDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", clawDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", clawDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", clawDistance.getDistance(DistanceUnit.INCH)));

            telemetry.update();
        }
    }
}
