/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.OpenCv;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name= "Left Auto")
public class LeftAuto extends LinearOpMode
{

    /*
    OpenCV / April Tags
     */
    OpenCvCamera phoneCam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // tag ids of signal sleeve
    int Left = 5;
    int Middle = 10;
    int Right = 15;

    AprilTagDetection tagOfInterest = null;


    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU               imu;
    Orientation angles;
    //Orientation             lastAngles = new Orientation();
    //double                  globalAngle, power = .30, correction;

    /*
     Mechanisms
      */
    // declare all of the servo and motor objects
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private Servo budsterupanddown;
    private Servo ElliottispotatoClaw;
    private Servo LiftRight;
    private Servo LiftLeft;

    // declare position variables
    double NewLiftPos;
    double LiftHeight;
    double FbFHeight;
    boolean LiftUpButtonIsPressed;
    boolean LiftDownButtonPressed;
    double MaxLiftHeight;
    double minLiftHeight;
    int NumLiftStops;
    double LiftLeftOffset;

    private static final double CATAPULT_IN = 0.79;
    private static final double CATAPULT_OUT = 0.265;


    private final static double ServoPosition = 0.5;
    private final static double ServoSpeed = 0.1;

    static final double DRIVE_SPEED = 0.45;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.40;     // Nominal half speed for better accuracy.
    static final double GRABBIT = 0.52;     // Nominal half speed for better accuracy.

    //PID control constants
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.025;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.007;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //OpenCV Pipeline
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        phoneCam.setPipeline(aprilTagDetectionPipeline);



        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        budsterupanddown = hardwareMap.get(Servo.class, "budsterupanddown");
        ElliottispotatoClaw = hardwareMap.get(Servo.class, "ElliottispotatoClaw");
        LiftRight = hardwareMap.get(Servo.class, "LiftRight");
        LiftLeft = hardwareMap.get(Servo.class, "LiftLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        budsterupanddown.setDirection(Servo.Direction.REVERSE);
        ElliottispotatoClaw.setDirection(Servo.Direction.REVERSE);
        LiftRight.setDirection(Servo.Direction.REVERSE);
        FbFHeight = 0.3;

        // Lift Variables
        LiftHeight = 0.11; // was 15 and raises up on init
        LiftLeftOffset = -0.022;
        MaxLiftHeight = 0.6;
        minLiftHeight = 0.12;
        NumLiftStops = 4;
        LiftDownButtonPressed = false;
        LiftUpButtonIsPressed = false;
        budsterupanddown.setPosition(0.5000000001);
        LiftLeft.setPosition(LiftLeftOffset + LiftHeight);
        LiftRight.setPosition(LiftHeight);
        NewLiftPos = LiftRight.getPosition();

        //Set motor modes
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Setting motor mode regarding encoders
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        telemetry.addData("Mode", "working...");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        telemetry.addData("Mode", "ready");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        //runtime.reset();
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            //telemetry.addData("Mode", "running");
            //telemetry.update();

            sleep(1000);

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Left || tag.id == Right || tag.id == Middle)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null){

            // do middle when nothing reads
            sleep(1000);
            telemetry.addLine("Op Mode Null - No Tag Read");
            telemetry.update();

            // Grab Cone
            ElliottispotatoClaw.setPosition(.42);
            sleep(800);

            // Lift 4bar
            //budsterupanddown.setPosition(.58); // .58 is
            budsterupanddown.setPosition(.90); // .58 is
            sleep(800);

            //Drive forward 1 inch so it doesn't drag the wall
            gyroDrive(0.30, inchesToTicks(1), 0);
            sleep(500);

            // Strafe over to line up with pole
            Drive(inchesToTicks(-9), inchesToTicks(9), inchesToTicks(9), inchesToTicks(-9), 0.5);
            sleep(500);

            // Drive forward towards pole
            gyroDrive(0.30, inchesToTicks(5.5), 0);
            sleep(500);

            // drop cone
            ElliottispotatoClaw.setPosition(.55);
            sleep(500);

            // backup from pole
            gyroDrive(0.30, inchesToTicks(-4), 0);
            sleep(500);

            // Strafe back to the middle of the parking zone
            Drive(inchesToTicks(14), inchesToTicks(-14), inchesToTicks(-14), inchesToTicks(14), 0.5);
            sleep(500);

            // Lower the 4 bar
            budsterupanddown.setPosition(.58); // .58 is
            sleep(500);

            // Drive forward to the middle of the zone
            //robot drives at 0.20 speed, 1000 encoder ticks, at 0 degrees
            gyroDrive(0.40, inchesToTicks(33.5), 0);

            // sleep until autonomous time is over
            //in case autonomous finishes before 30 seconds, the while loop won't run again
            sleep(100000);

        } else if (tagOfInterest.id == Middle) {

            // do middle
            sleep(1000);
            telemetry.addLine("Op Mode Middle - Read Tag ID 10");
            telemetry.update();

            // Grab Cone
            ElliottispotatoClaw.setPosition(.42);
            sleep(800);

            // Lift 4bar
            //budsterupanddown.setPosition(.58); // .58 is
            budsterupanddown.setPosition(.90); // .58 is
            sleep(800);

            //Drive forward 1 inch so it doesn't drag the wall
            gyroDrive(0.30, inchesToTicks(1), 0);
            sleep(500);

            // Strafe over to line up with pole
            Drive(inchesToTicks(-9), inchesToTicks(9), inchesToTicks(9), inchesToTicks(-9), 0.5);
            sleep(500);

            // Drive forward towards pole
            gyroDrive(0.30, inchesToTicks(5.5), 0);
            sleep(500);

            // drop cone
            ElliottispotatoClaw.setPosition(.55);
            sleep(500);

            // backup from pole
            gyroDrive(0.30, inchesToTicks(-4), 0);
            sleep(500);

            // Strafe back to the middle of the parking zone
            Drive(inchesToTicks(14), inchesToTicks(-14), inchesToTicks(-14), inchesToTicks(14), 0.5);
            sleep(500);

            // Lower the 4 bar
            budsterupanddown.setPosition(.58); // .58 is
            sleep(500);

            // Drive forward to the middle of the zone
            //robot drives at 0.20 speed, 1000 encoder ticks, at 0 degrees
            gyroDrive(0.40, inchesToTicks(33.5), 0);

            // sleep until autonomous time is over
            //in case autonomous finishes before 30 seconds, the while loop won't run again
            sleep(100000);

        } else if (tagOfInterest.id == Right) {

            // do right
            sleep(1000);
            telemetry.addLine("Op Mode Middle - Read Tag ID 10");
            telemetry.update();

            // Grab Cone
            ElliottispotatoClaw.setPosition(.42);
            sleep(800);

            // Lift 4bar
            //budsterupanddown.setPosition(.58); // .58 is
            budsterupanddown.setPosition(.90); // .58 is
            sleep(800);

            //Drive forward 1 inch so it doesn't drag the wall
            gyroDrive(0.30, inchesToTicks(1), 0);
            sleep(500);

            // Strafe over to line up with pole
            Drive(inchesToTicks(-9), inchesToTicks(9), inchesToTicks(9), inchesToTicks(-9), 0.5);
            sleep(500);

            // Drive forward towards pole
            gyroDrive(0.30, inchesToTicks(5.5), 0);
            sleep(500);

            // drop cone
            ElliottispotatoClaw.setPosition(.55);
            sleep(500);

            // backup from pole
            gyroDrive(0.30, inchesToTicks(-4), 0);
            sleep(500);

            // Strafe back to the middle of the parking zone
            Drive(inchesToTicks(-14), inchesToTicks(14), inchesToTicks(14), inchesToTicks(-14), 0.5);
            sleep(500);

            // Lower the 4 bar
            budsterupanddown.setPosition(.58); // .58 is
            sleep(500);

            // Drive forward to the middle of the zone
            //robot drives at 0.20 speed, 1000 encoder ticks, at 0 degrees
            gyroDrive(0.40, inchesToTicks(33.5), 0);

            // sleep until autonomous time is over
            //in case autonomous finishes before 30 seconds, the while loop won't run again
            sleep(100000);

        } else if (tagOfInterest.id == Left) {

            // do left
            sleep(1000);
            telemetry.addLine("Op Mode Middle - Read Tag ID 10");
            telemetry.update();

            // Grab Cone
            ElliottispotatoClaw.setPosition(.42);
            sleep(800);

            // Lift 4bar
            //budsterupanddown.setPosition(.58); // .58 is
            budsterupanddown.setPosition(.90); // .58 is
            sleep(800);

            //Drive forward 1 inch so it doesn't drag the wall
            gyroDrive(0.30, inchesToTicks(1), 0);
            sleep(500);

            // Strafe over to line up with pole
            Drive(inchesToTicks(-9), inchesToTicks(9), inchesToTicks(9), inchesToTicks(-9), 0.5);
            sleep(500);

            // Drive forward towards pole
            gyroDrive(0.30, inchesToTicks(5.5), 0);
            sleep(500);

            // drop cone
            ElliottispotatoClaw.setPosition(.55);
            sleep(500);

            // backup from pole
            gyroDrive(0.30, inchesToTicks(-4), 0);
            sleep(500);

            // Strafe back to the middle of the parking zone
            Drive(inchesToTicks(41.5), inchesToTicks(-41.5), inchesToTicks(-41.5), inchesToTicks(41.5), 0.5);
            sleep(500);

            // Lower the 4 bar
            budsterupanddown.setPosition(.58); // .58 is
            sleep(500);

            // Drive forward to the middle of the zone
            //robot drives at 0.20 speed, 1000 encoder ticks, at 0 degrees
            gyroDrive(0.40, inchesToTicks(33.5), 0);

            // sleep until autonomous time is over
            //in case autonomous finishes before 30 seconds, the while loop won't run again
            sleep(100000);

        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


    public int inchesToTicks( double inches ) {
        return (int)(inches * 1500 / 33.5);
    }
    /**
     * Robot drives in straight line and corrects drift using gyro sensor with PID control.
     *
     * @param speed    The speed that the robot drives at (always positive)
     * @param distance The distance, in encoder ticks, that the robot drives
     *                 (positive goes forward, negative goes backward)
     * @param angle    The angle that the robot drives at (must match any previous)
     *                 turn angles) [-180, 180]
     */
    public void gyroDrive ( double speed, int distance, double angle){
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontRight.setTargetPosition(distance);
            FrontLeft.setTargetPosition(distance);
            BackLeft.setTargetPosition(distance);
            BackRight.setTargetPosition(distance);

            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FrontRight.setPower(speed);
            FrontLeft.setPower(speed);
            BackRight.setPower(speed);
            BackLeft.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (FrontRight.isBusy() && FrontLeft.isBusy() && BackLeft.isBusy() && BackRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                FrontLeft.setPower(leftSpeed);
                BackLeft.setPower(leftSpeed);
                FrontRight.setPower(rightSpeed);
                BackRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            FrontRight.setPower(0);
            BackRight.setPower(0);
            FrontLeft.setPower(0);
            BackLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn ( double speed, double angle){

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }


    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold ( double speed, double angle, double holdTime){

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        FrontRight.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        BackLeft.setPower(0);

    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading ( double speed, double angle, double PCoeff){

        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        FrontLeft.setPower(leftSpeed);
        BackLeft.setPower(leftSpeed);
        FrontRight.setPower(rightSpeed);
        BackRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Error", error);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError ( double targetAngle){

        double robotError;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // calculate error in -179 to +180 range  (
        robotError = angles.firstAngle - targetAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer ( double error, double PCoeff){
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * Robot moves (strafe, turn, straight line) given distance in encoder ticks for each drive
     * motor and speed without correcting for drift.
     *
     * @param frontRightDistance Distance in encoder ticks (positive or negative values
     *                           determines whether the robt drives forward or backward,
     *                           turns, or strafes)
     * @param frontLeftDistance  ^^
     * @param rearRightDistance  ^^
     * @param rearLeftDistance   ^^
     * @param speed              Speed that each motor moves
     */
    public void Drive ( int frontRightDistance, int frontLeftDistance, int rearRightDistance,
                        int rearLeftDistance, double speed){


        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRight.setTargetPosition(frontRightDistance);
        FrontLeft.setTargetPosition(frontLeftDistance);
        BackRight.setTargetPosition(rearRightDistance);
        BackLeft.setTargetPosition(rearLeftDistance);

        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontRight.setPower(speed);
        FrontLeft.setPower(speed);
        BackRight.setPower(speed);
        BackLeft.setPower(speed);


        while (FrontRight.isBusy() && FrontLeft.isBusy() && BackRight.isBusy() && BackLeft.isBusy() && opModeIsActive()) {
            telemetry.addData("FrontRightPosition", FrontRight.getCurrentPosition());
            telemetry.addData("FrontLeftPosition", FrontLeft.getCurrentPosition());
            telemetry.update();
        }

        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);
        BackLeft.setPower(0);

        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
    }

}