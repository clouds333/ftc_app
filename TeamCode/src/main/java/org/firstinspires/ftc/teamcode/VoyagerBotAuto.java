/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name 
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "Autonoumous_test: autonomous_test", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class VoyagerBotAuto extends LinearOpMode {

    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     *
     */
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    HardwarePushbot robot  = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    int newLeftTarget;
    int newRightTarget;
    static final double     TURN_SPEED              = 0.5;
    static final double     FORWARD_SPEED = 0.6;
    static final double     DRIVE_SPEED              = 0.5;
    
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    
    public void vuMarkMode() {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AThYzfT/////AAAAGckUHRIsekOBmxU6KcvbHqUrRkjaM/ix//1zO1TT5wcUh9w+0kjzOW4gAKMPqHRRb+r7GbuHMJyNqhWSAJCY6AFXpVYW+/t1q1pp4w9+OGmdh8eO7LIISlt+H+n3HCO851UHC/t2FtS3ZrAT9IbBAfccXObdXx4ZsgPAeK5NpeB2yoAbZmsaVi8IJDSsEznjLSP/zOGb4AxgMT4rpGBjy0784870zE93Dj0Dlt7laBcxZip5y1mRSI3Q2MJTKzJ2iHvJOwPT4T3fRUN8aOiDzHSz5YEkHtS7kzX2EWlu4ENk7E7NlXDaELabIsY1KRTEaeoD4xFNNYCbBiRV7ydwGkiwSu6V9hfDLEuWax9/GPYo";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }
    
    
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    
    public void init2() {
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorLeftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            //newLeftTarget = robot.motorLeftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            //newRightTarget = robot.motorRightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            
            robot.motorLeftFront.setTargetPosition(newLeftTarget);
            //robot.motorLeftBack.setTargetPosition(newLeftTarget);
            //robot.motorRightFront.setTargetPosition(newRightTarget);
            robot.motorRightBack.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorLeftFront.setPower(Math.abs(speed));
            robot.motorLeftBack.setPower(Math.abs(speed));
            robot.motorRightFront.setPower(Math.abs(speed));
            robot.motorRightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.motorLeftFront.isBusy() && robot.motorRightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d %7d %7d",
                                            robot.motorLeftFront.getCurrentPosition(),
                                            robot.motorLeftBack.getCurrentPosition(),
                                            robot.motorRightFront.getCurrentPosition(),
                                            robot.motorRightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorLeftFront.setPower(0);
            robot.motorLeftBack.setPower(0);
            robot.motorRightFront.setPower(0);
            robot.motorRightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    @Override
    public void runOpMode() {
        boolean isJewelNotDetected = true;
        
        robot.init(hardwareMap);
        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorSensor");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        int redValue = 0;
        int blueValue = 0;
        
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        //robot.colorServo.setPosition(1.0);
        init2();
        
        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
                    
            // forwards

            //robot.motorLeftFront.setPower(FORWARD_SPEED);
            //robot.motorLeftBack.setPower(FORWARD_SPEED);
            //robot.motorRightFront.setPower(FORWARD_SPEED);
            //robot.motorRightBack.setPower(FORWARD_SPEED);
            //runtime.reset();
            //while (opModeIsActive() && (runtime.seconds() < 0.5)) {
              //  telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                //telemetry.update();
            //

            // backwards
            
            //robot.motorLeftFront.setPower(-FORWARD_SPEED);
            //robot.motorLeftBack.setPower(-FORWARD_SPEED);
            //robot.motorRightFront.setPower(-FORWARD_SPEED);
            //robot.motorRightBack.setPower(-FORWARD_SPEED);
            //runtime.reset();
            //while (opModeIsActive() && (runtime.seconds() < 0.5)) {
              //  telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                //telemetry.update();
            //}
            // Send telemetry message to signify robot waiting;
        
            
            if (isJewelNotDetected) {
                robot.colorServo.setPosition(1.0);
            }


            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            
            redValue = sensorColor.red();
            blueValue = sensorColor.blue();
            
            //Sensing if it is Red
            if (redValue >80 && redValue > blueValue * 1.8) {
                isJewelNotDetected = false;
                telemetry.addData("red jewel detected ", redValue );
                telemetry.update();
                /*drive forward
                encoderDrive(DRIVE_SPEED,  1,  1, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                robot.colorServo.setPosition(0.0);
                encoderDrive(DRIVE_SPEED,  12,  12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
*/
                //Drive backwards
                robot.motorLeftFront.setPower(-FORWARD_SPEED-0.3);
                robot.motorLeftBack.setPower(-FORWARD_SPEED-0.3);
                robot.motorRightFront.setPower(-FORWARD_SPEED-0.3);
                robot.motorRightBack.setPower(-FORWARD_SPEED-0.3);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                
                };
                //Raising Arm back up
                robot.colorServo.setPosition(0.0);
                while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                //Stop motors
                robot.motorLeftFront.setPower(0);
                robot.motorLeftBack.setPower(0);
                robot.motorRightFront.setPower(0);
                robot.motorRightBack.setPower(0);
                    
            
            //Blue
            } else if (blueValue > 80 && blueValue > redValue * 1.8) {
                isJewelNotDetected = false;
                telemetry.addData("blue jewel detected ", blueValue );
                telemetry.update();
                
                robot.motorLeftFront.setPower(-FORWARD_SPEED+0.3);
                robot.motorLeftBack.setPower(-FORWARD_SPEED+0.3);
                robot.motorRightFront.setPower(-FORWARD_SPEED+0.3);
                robot.motorRightBack.setPower(-FORWARD_SPEED+0.3);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                };
                
                
                robot.motorLeftFront.setPower(0);
                robot.motorLeftBack.setPower(0);
                robot.motorRightFront.setPower(0);
                robot.motorRightBack.setPower(0);
                robot.colorServo.setPosition(0.0);
                while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                /*
                encoderDrive(DRIVE_SPEED,  -5,  -5, 1.0);  // S1: Forward 47 Inches with 5 Sec timeout
                
                };
                */

            } else {
                telemetry.addData("No color jewel detected", redValue );
                telemetry.update();
            }
        }

    }
    
}
