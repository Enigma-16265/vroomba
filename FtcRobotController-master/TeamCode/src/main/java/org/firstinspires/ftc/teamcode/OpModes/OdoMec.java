package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="OdoMec")


public class OdoMec extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Motors
    //Drivetrain
    private DcMotor rightFront; //front right 0
    private DcMotor leftFront; //front left 2
    private DcMotor rightRear; //rear right 1
    private DcMotor leftRear; //rear left 3

    //Encoders
    private DcMotor verticalRight; // 1
    private DcMotor verticalLeft; // 3
    private DcMotor horizontal; // 0

    //Mechanism motors
    private DcMotor turret;
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    //Servos
    private Servo odoRetractor;
    private Servo flipOut;
    private Servo finger;
    private Servo claw;
    private Servo clawLinkage;

    //Magnetic Switches
    private RevTouchSensor slideMag;
    private RevTouchSensor homeMag;

    //Distance sensor
    private DistanceSensor clawDistance;

    BNO055IMU imu;                // Additional Gyro device
    Orientation angles;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables
        //Drive motors
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");

        //Encoders
        verticalRight = hardwareMap.get(DcMotor.class, "rightRear");
        verticalLeft = hardwareMap.get(DcMotor.class, "leftRear");
        horizontal = hardwareMap.get(DcMotor.class, "rightFront");

        //Mechanism motors
        turret = hardwareMap.get(DcMotor.class, "turret");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        //Servos
        odoRetractor = hardwareMap.get(Servo.class, "odoRetractor");
        flipOut = hardwareMap.get(Servo.class, "flipOut");
        finger = hardwareMap.get(Servo.class, "finger");
        claw = hardwareMap.get(Servo.class, "claw");
        clawLinkage = hardwareMap.get(Servo.class, "clawLinkage");

        //Magnetic switches
        slideMag = hardwareMap.get(RevTouchSensor.class, "slideMag");
        homeMag = hardwareMap.get(RevTouchSensor.class, "homeMag");

        //Distance sensor
        clawDistance = hardwareMap.get(DistanceSensor.class, "clawDistance");
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)clawDistance;

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);

        //Set motor modes
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //Set servo positions



        telemetry.addData("Status", "OdoMec2 is ready to run!");
        telemetry.update();

        //Wait for press play
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Drivetrain
            double forward = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(forward)+Math.abs(strafe)+Math.abs(turn), 1);

            double rightFrontPower = (forward - strafe - turn) / denominator;
            double leftFrontPower = (forward + strafe + turn) / denominator;
            double rightRearPower = (forward + strafe - turn) / denominator;
            double leftRearPower = (forward - strafe + turn) / denominator;

            if (gamepad1.left_bumper) {
                rightFrontPower = Range.clip(rightFrontPower, -0.4, 0.4);
                leftFrontPower = Range.clip(leftFrontPower, -0.4, 0.4);
                rightRearPower = Range.clip(rightRearPower, -0.4, 0.4);
                leftRearPower = Range.clip(leftRearPower, -0.4, 0.4);
            } else {
                rightFrontPower = Range.clip(rightFrontPower, -0.8, 0.8);
                leftFrontPower = Range.clip(leftFrontPower, -0.8, 0.8);
                rightRearPower = Range.clip(rightRearPower, -0.8, 0.8);
                leftRearPower = Range.clip(leftRearPower, -0.8, 0.8);
            }


            rightFront.setPower(rightFrontPower);
            leftFront.setPower(leftFrontPower);
            rightRear.setPower(rightRearPower);
            leftRear.setPower(leftRearPower);

            telemetry.addData("Status", "Run " + runtime.toString());
            telemetry.addData("Motors", "forward (%.2f), strafe (%.2f),turn (%.2f)" , forward, strafe, turn);
            telemetry.update();



            //Mechanisms
            //slideMag.isPressed()

            //clawDistance.getDistance();
            telemetry.addData("deviceName",clawDistance.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", clawDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", clawDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", clawDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", clawDistance.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();

            if (gamepad2.a) {
                clawLinkage.setPosition(0.5);
            }
            //else

            if (gamepad2.left_bumper) {
                turret.setPower(0.2);
            } else {
                turret.setPower(0);
            }

            idle();

        }
    }
}
