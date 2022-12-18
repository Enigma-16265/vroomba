package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MecDrive")


public class OdoBotTeleOp extends LinearOpMode {
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

    BNO055IMU imu;                // Additional Gyro device
    Orientation angles;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables
        //Motors
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");

        //Encoders
        verticalRight = hardwareMap.get(DcMotor.class, "rightRear");
        verticalLeft = hardwareMap.get(DcMotor.class, "leftRear");
        horizontal = hardwareMap.get(DcMotor.class, "rightFront");

        //Set motor directions
        /////////////////////////////////////////////////////////////////////////////////
        //rightFront.setDirection(DcMotor.Direction.FORWARD);
        //leftFront.setDirection(DcMotor.Direction.REVERSE);
        //rightRear.setDirection(DcMotor.Direction.FORWARD);
        //leftRear.setDirection(DcMotor.Direction.REVERSE);

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

        telemetry.addData("Status", "OdoBot is ready to run!");
        telemetry.update();

        //Wait for press play
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Drivetrain
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;

            double wheelFrontRightPower = pivot - vertical - horizontal;
            double wheelFrontLeftPower = pivot - vertical + horizontal;
            double wheelRearRightPower = -pivot - vertical + horizontal;
            double wheelRearLeftPower = -pivot - vertical - horizontal;

            if (gamepad1.left_bumper) {
                wheelFrontRightPower = Range.clip(wheelFrontRightPower, -0.3, 0.3);
                wheelFrontLeftPower = Range.clip(wheelFrontLeftPower, -0.3, 0.3);
                wheelRearRightPower = Range.clip(wheelRearRightPower, -0.3, 0.3);
                wheelRearLeftPower = Range.clip(wheelRearLeftPower, -0.3, 0.3);
            } else {
                wheelFrontRightPower = Range.clip(wheelFrontRightPower, -0.8, 0.8);
                wheelFrontLeftPower = Range.clip(wheelFrontLeftPower, -0.8, 0.8);
                wheelRearRightPower = Range.clip(wheelRearRightPower, -0.8, 0.8);
                wheelRearLeftPower = Range.clip(wheelRearLeftPower, -0.8, 0.8);
            }


            rightFront.setPower(wheelFrontRightPower);
            leftFront.setPower(wheelRearRightPower);
            rightRear.setPower(wheelFrontLeftPower);
            leftRear.setPower(wheelRearLeftPower);
        }
    }
}
