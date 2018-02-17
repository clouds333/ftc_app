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
import java.util.EnumSet;
import com.qualcomm.robotcore.util.Range;
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
@Autonomous(name = "Voyagerbot: Autonomous", group = "Sensor")
@Disabled                            // Comment this out to add to the opmode list
public class VoyagerBotAuto extends VoyagerBotAutoTest {

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
    private boolean grabGlyph = true;
    //HardwareVoyagerbot robot  = new HardwareVoyagerbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    //static final double     TURN_SPEED              = 0.3;
    //static final double     FORWARD_SPEED           = 0.6;
    //static final double     DRIVE_SPEED             = 0.5;
    //static final double     FAST_SPEED              = 0.7;
    //static final double     SLOW_SPEED              = 0.10;
    static double COLOR_SERVO_SPEED = 0.05;
    static final double     CRYPTO_BOX_INCHES       = 7.63;

    public enum Orientation {
       BLUE_1,
       BLUE_2,
       RED_1,
       RED_2;
       private static final int amount = EnumSet.allOf(Orientation.class).size();
       private static Orientation[] val = new Orientation[amount];
       static{ for(Orientation q:EnumSet.allOf(Orientation.class)){ val[q.ordinal()]=q; } }
       public static Orientation fromInt(int i) { return val[i]; }
       public Orientation next() { return fromInt((ordinal()+1)%amount); }
    }

    public Orientation startOrientation = Orientation.BLUE_1;
    
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables = null;
    VuforiaTrackable relicTemplate = null;
    RelicRecoveryVuMark detectedVuMark = RelicRecoveryVuMark.UNKNOWN;
    RelicRecoveryVuMark defaultLocation = RelicRecoveryVuMark.RIGHT;  // default to this if unknown
    boolean grabSecond = false;
    
    public void initVuMark() {
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
      relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
      relicTemplate = relicTrackables.get(0);
      relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    public RelicRecoveryVuMark detectVuMark() {

    
      relicTrackables.activate();

      // reset the timeout
      if (opModeIsActive()) {
          runtime.reset();
          while (opModeIsActive()  && (runtime.seconds() < 2.0)) {
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
        
                    return vuMark;
                }
                else {
                    telemetry.addData("VuMark", "not visible");
                }
        
                telemetry.update();
          }
      }
      return defaultLocation;
    }
    
    public void initOrientation() {
      startOrientation = Orientation.BLUE_1;
    }
    
    public void initRobot() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        robot.motorLeftBack.setDirection(DcMotor.Direction.FORWARD);
        
        robot.motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        robot.motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        
        robot.motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    
    public void doGrabGlyph(boolean isClose) {
        double clawOffset = 0;

        if (isClose) {
            clawOffset = -0.2;
        } else {
            clawOffset = 0.2;
        }
        clawOffset = Range.clip(clawOffset, -0.12, 0.08);
        robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
        robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);
        sleep (300);

    }
    


    public void init_loop2() {
        // read the gamepad
        if (gamepad1.right_bumper) {
          // switch the Orientation
          startOrientation = startOrientation.next();

        }
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            /* Found an instance of the template. In the actual game, you will probably
             * loop until this condition occurs, then move on to act accordingly depending
             * on which VuMark was visible. */
             detectedVuMark = vuMark;
            telemetry.addData("VuMark", "%s visible", vuMark);
        }
        telemetry.addData("Selected Orientation: ", startOrientation);
        telemetry.addData("VuMark Found: ", vuMark);
        telemetry.update();
    }
    
    @Override
    public void runOpMode() {
        boolean isJewelNotDetected = true;
        
        robot.init(hardwareMap, true);
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

        initOrientation();
        initRobot();
        initVuMark();
        initGyro();
        robot.colorServo.setPosition(0.0);
       // robot.leftArm.setPosition();
        
        relicTrackables.activate();
        
        // wait for the start button to be pressed.
        //waitForStart();
        AutoTransitioner.transitionOnStop(this, "VoyagerBotTeleop");
        // AutoTransitioner used before waitForStart()
        while (!opModeIsActive() && !isStopRequested()) {
           init_loop2();
           // waitOneFullHardwareCycle();
           sleep(50); // temporary fix to waitXXXHardwareCycle bug?
        }
        
        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.

        runtime.reset();
        while (opModeIsActive()) {
            if (isJewelNotDetected) {
                telemetry.addData("moving colorServo %s", "");
                telemetry.update();     
                //moveColorServo(1);
                robot.colorServo.setPosition(1.0);                
                doGrabGlyph(false);
                encoderLift(0.7, 300, 1);  
                //sleep(1000);
            }

            ElapsedTime colorDetectTime = new ElapsedTime();
            while (opModeIsActive() && isJewelNotDetected == true && colorDetectTime.seconds() <= 2) {
                // convert the RGB values to HSV values.
                // multiply by the SCALE_FACTOR.
                // then cast it back to int (SCALE_FACTOR is a double)
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                redValue = sensorColor.red();
                blueValue = sensorColor.blue();                
                //Red
                if (redValue > blueValue * 1.2) { 
                    isJewelNotDetected = false;
                    int direction = knockOffJewel(true, startOrientation);
                    driveToCryptoBox(startOrientation, detectedVuMark, direction);                    
                
                } else if (blueValue > redValue * 1.2) {
                //Blue    
                    isJewelNotDetected = false;
                    int direction = knockOffJewel(false, startOrientation);
                    driveToCryptoBox(startOrientation, detectedVuMark, direction);
                /*} else if (runtime.seconds() > 2) {
                // we did not detect anything, just go
                    driveToCryptoBox(startOrientation, detectedVuMark, 0);*/                  
                } else {
                    telemetry.addData("No color jewel detected", redValue);
                    telemetry.update();
                }
            }
            if (isJewelNotDetected == true) {
                isJewelNotDetected = false;   
                noJewel(startOrientation);
                driveToCryptoBox(startOrientation, detectedVuMark, 0);                  
            }
            
        }

    }
    
    public void moveColorServo(int desiredPosition){
        if (desiredPosition == 1) {
            while (opModeIsActive() && robot.colorServo.getPosition() < desiredPosition) {
                robot.colorServo.setPosition(robot.colorServo.getPosition() + COLOR_SERVO_SPEED);
                sleep(50);
            }
        } else {
            while (opModeIsActive() && robot.colorServo.getPosition() > desiredPosition) {
                robot.colorServo.setPosition(robot.colorServo.getPosition() - COLOR_SERVO_SPEED);
                sleep(50);
            }            
        }
        robot.colorServo.setPosition(desiredPosition);
        sleep(300);
    }
    
    public void noJewel(Orientation orientation) {
        if (orientation == Orientation.BLUE_1 || orientation == Orientation.BLUE_2) {   
            moveColorServo(0);
            encoderDrive(SLOW_SPEED, -3, -3, 2);
        } else {
            moveColorServo(0);                
            encoderDrive(SLOW_SPEED, 3, 3, 1.5); // go forwards to knock jewel off
        }
    }
    
    // return 0 if forward, 1 if backward
    public int knockOffJewel(boolean redDetected, Orientation orientation) {
        if (orientation == Orientation.BLUE_1 || orientation == Orientation.BLUE_2) {
          if (redDetected) {
            // we are blue, red detected, go backward and knockOffJewel
            encoderDrive(SLOW_SPEED, 2, 2, 2); // go backwards to knock jewel off
            moveColorServo(0);
            //robot.colorServo.setPosition(0.0); //lift arm back up
            //sleep(1000);
            encoderDrive(SLOW_SPEED, -6.5, -6.5, 3.0); // go to original position
            return 1;
          } else{
            // we are blue and blue detected, go forward to knockOffJewel
            encoderDrive(SLOW_SPEED, -3, -3, 2); // go backwards to knock jewel off
            moveColorServo(0);
            //robot.colorServo.setPosition(0.0); //lift arm back up
            //sleep(1000);
            return 0;
          }
        } else {
          // must be red alliance
          if (redDetected) {
            encoderDrive(SLOW_SPEED, -1.5, -1.5, 1.5); // go backwards to knock jewel off
            moveColorServo(0);            
            //robot.colorServo.setPosition(0.0); //lift arm back up
            //sleep(1000);
            encoderDrive(SLOW_SPEED, 4, 4, 1.0); // go to original position
            return 1;
          } else {
            encoderDrive(SLOW_SPEED, 3, 3, 1.5); // go forwards to knock jewel off
            moveColorServo(0);            
            //robot.colorServo.setPosition(0.0); //lift arm back up
            //sleep(1000);
            return 0;
          }
        }
    }

    public void driveToCryptoBox(Orientation orientation, RelicRecoveryVuMark vuMark, int direction) {
        double cryptoOffset = 0;
        if (vuMark == RelicRecoveryVuMark.UNKNOWN){
            vuMark = RelicRecoveryVuMark.RIGHT;
        }
        if (orientation == Orientation.BLUE_1) {
          if (vuMark == RelicRecoveryVuMark.RIGHT ) {
              cryptoOffset = -4.5;
          } if (vuMark == RelicRecoveryVuMark.CENTER ) {
              cryptoOffset = -12.5;
          } else if (vuMark == RelicRecoveryVuMark.LEFT ) {
              cryptoOffset = -20.5;
          }             
          blue_1_forward_internal(false, cryptoOffset, direction);
        } else if (orientation == Orientation.BLUE_2) { 
          if (vuMark == RelicRecoveryVuMark.RIGHT ) {
              cryptoOffset = 1.5;
          } if (vuMark == RelicRecoveryVuMark.CENTER ) {
              cryptoOffset = 10;
          } else if (vuMark == RelicRecoveryVuMark.LEFT ) {
              cryptoOffset = 17.5;
          }               
          blue_2_forward_internal(false, cryptoOffset, direction);          
        } else if (orientation == Orientation.RED_1) {
          if (vuMark == RelicRecoveryVuMark.LEFT ) {
              cryptoOffset = 5;
          } if (vuMark == RelicRecoveryVuMark.CENTER ) {
              cryptoOffset = 12.5;
          } else if (vuMark == RelicRecoveryVuMark.RIGHT ) {
              cryptoOffset = 20;
          }              
          red_1_forward_internal(grabSecond, cryptoOffset, direction);
        } else if (orientation == Orientation.RED_2) {   
          if (vuMark == RelicRecoveryVuMark.LEFT ) {
              cryptoOffset = 4.5;
          } if (vuMark == RelicRecoveryVuMark.CENTER ) {
              cryptoOffset = 12;
          } else if (vuMark == RelicRecoveryVuMark.RIGHT ) {
              cryptoOffset = 19;
          }               
          red_2_forward_internal(false, cryptoOffset, direction);          
        }

    }  
    
}
