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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Voyagerbot: Single Test", group="Voyagerbot")
//@Disabled
public class VoyagerBotAutoTest extends GyroAuto {

    /* Declare OpMode members. */
    // HardwareVoyagerbot         robot   = new HardwareVoyagerbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     TURN_SPEED              = 0.6;
    static final double     FORWARD_SPEED           = 0.6;
    static final double     DRIVE_SPEED             = 0.5;
    static final double     FAST_SPEED              = 0.7;
    static final double     SLOW_SPEED              = 0.2;
        
    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, true);
        initGyro();
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

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d, %7d %7d",
                          robot.motorLeftFront.getCurrentPosition(),
                          robot.motorLeftBack.getCurrentPosition(),
                          robot.motorRightFront.getCurrentPosition(),
                          robot.motorRightBack.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        doGrabGlyph(false);
        encoderLift(0.8, 400, 2);  
        red_1_forward();
        //red_1_forward();
        //red_get_off_beam(0);
        //encoderDrive(SLOW_DRIVE_SPEED, 20, 20, 20, 20, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   -18, -18, 18, 18, 4.0);  // S2: Turn Left 12 Inches with 4 Sec timeout
        //encoderDrive(SLOW_DRIVE_SPEED, 20, 20, 20, 20, 5.0); 
        //encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    void blue_1_forward() {
        double bd = -4.5;
        encoderDrive(DRIVE_SPEED, bd, bd, 5.0); // go forwards    
        gyroTurn(TURN_SPEED, -90);
        gyroTurn(TURN_SPEED, -90);        
        encoderDrive(FAST_SPEED, 5, 5, 5.0); // go forwards
        doGrabGlyph(true);
        encoderDrive(DRIVE_SPEED, 4, 4, 3.0); // go forwards   
        encoderDrive(FAST_SPEED, -4, -4, 2.0); // backwards  
        doGrabGlyph(false);        
        encoderDrive(DRIVE_SPEED, 5, 5, 3.0); // go forwards   
        encoderDrive(FAST_SPEED, -4, -4, 2.0); // backwards  
    }
    
    void blue_1_forward_internal(boolean grabSecond, double bd, int direction) {
        if (direction == 0)
            encoderDrive(SLOW_SPEED, -15, -15, 8.0);
        else
            encoderDrive(SLOW_SPEED, -19, -19, 8.0);
        
        // go forward a little to hit the balance beam
        encoderDrive(DRIVE_SPEED, 2, 2, 1.0);
        //double bd = -5;
        //bd = bd - 8;
        encoderDrive(FAST_SPEED, bd, bd, 5.0); // go forwards    
        gyroTurn(TURN_SPEED, -90);
        gyroTurn(TURN_SPEED, -90);        
        encoderDrive(FAST_SPEED, 5, 5, 5.0); // go forwards
        doGrabGlyph(true);
        encoderDrive(DRIVE_SPEED, 4, 4, 3.0); // go forwards   
        encoderDrive(FAST_SPEED, -5, -5, 2.0); // backwards  
        encoderLift(0.6, 0, 1.5);              // lower lift          
        doGrabGlyph(false);        
        encoderDrive(DRIVE_SPEED, 6, 6, 3.0); // go forwards   
        encoderDrive(FAST_SPEED, -4, -4, 2.0); // backwards          
    }
    
    void blue_2_forward() {
        double bd = 9;
        encoderDrive(DRIVE_SPEED, -3, -3, 2.0); // go forwards         
        gyroTurn(TURN_SPEED, 90 );
        gyroTurn(TURN_SPEED, 90 );        
        encoderDrive(FAST_SPEED, bd, bd, 2.0); // go forwards 
        initGyro();
        gyroTurn(TURN_SPEED, 90 );
        gyroTurn(TURN_SPEED, 90 );        
        //encoderDrive(DRIVE_SPEED, 2, 2, 2.0); // go forwards
        doGrabGlyph(true);
        encoderDrive(DRIVE_SPEED, 5, 5, 1.0); // go forwards
        encoderDrive(FAST_SPEED, -5, -5, 2.0); // go backwards  
        doGrabGlyph(false);
        encoderDrive(FAST_SPEED, 6, 6, 2.0); // go forwards   
        encoderDrive(FAST_SPEED, -4, -4, 2.0); // go forwards   
    }
    
    void blue_2_forward_internal(boolean grabSecond, double bd, int direction) {
        if (direction == 0)
            encoderDrive(SLOW_SPEED, -15, -15,  8.0);
        else
            encoderDrive(SLOW_SPEED, -19, -19, 8.0);        
        // go forward a little to hit the balance beam
        encoderDrive(DRIVE_SPEED, 2, 2, 1.0);   
        
        encoderDrive(DRIVE_SPEED, -3, -3, 2.0); // go forwards         
        gyroTurn(TURN_SPEED, 90 );
        gyroTurn(TURN_SPEED, 90 );        
        encoderDrive(FAST_SPEED, bd, bd, 2.0); // go forwards 
        //initGyro();
        resetAngle();
        gyroTurn(TURN_SPEED, 90 );
        gyroTurn(TURN_SPEED, 90 );        
        encoderDrive(DRIVE_SPEED, 2, 2, 2.0); // go forwards
        doGrabGlyph(true);
        encoderDrive(DRIVE_SPEED, 5, 5, 1.0); // go forwards
        encoderDrive(FAST_SPEED, -6, -6, 2.0); // go backwards  
        encoderLift(0.6, 0, 1.5);              // lower lift          
        doGrabGlyph(false);
        encoderDrive(FAST_SPEED, 7, 7, 2.0); // go forwards   
        encoderDrive(FAST_SPEED, -4, -4, 2.0); // go forwards          
    }    


    void red_2_forward() {
        double bd = 4.5;
        encoderDrive(DRIVE_SPEED, 4, 4, 1.5); // get off balancing beam          
        gyroTurn(TURN_SPEED, 90 ); //turn 90 degrees
        gyroTurn(TURN_SPEED, 90 ); 
        encoderDrive(DRIVE_SPEED, bd, bd, 3.0); // go forwards 
        //initGyro();
        resetAngle();
        gyroTurn(TURN_SPEED, -90 ); // turn back to pictograph
        gyroTurn(TURN_SPEED, -90 );        
        encoderDrive(FAST_SPEED, 1, 1, 3.0); // go forwards
        doGrabGlyph(true);
        encoderDrive(DRIVE_SPEED, 4, 4, 2.0); // go forwards
        encoderDrive(FAST_SPEED, -5, -5,  2.0); // go forwards
        doGrabGlyph(false);
        encoderDrive(FAST_SPEED, 6, 6, 2.0); // go forwards   
        encoderDrive(FAST_SPEED, -5, -5, 2.0); // go forwards         
    }
    
    void red_2_forward_internal(boolean grabSecond, double bd, int direction) {
        if (direction == 0)
            encoderDrive(SLOW_SPEED, 15, 15, 8.0);
        else
            encoderDrive(SLOW_SPEED, 20, 20, 8.0);       
        encoderDrive(DRIVE_SPEED, -3, -3, 1.0); // backup to straighten up  
        encoderDrive(DRIVE_SPEED, 3, 3, 1.5); // get off balancing beam          
        gyroTurn(TURN_SPEED, 90 ); //turn 90 degrees
        gyroTurn(TURN_SPEED, 90 ); 
        encoderDrive(FAST_SPEED, bd, bd, 2.0); // go forwards 
        //initGyro();
        resetAngle();
        gyroTurn(TURN_SPEED, -90 ); // turn back to pictograph
        gyroTurn(TURN_SPEED, -90 );        
        encoderDrive(FAST_SPEED, 2, 2, 3.0); // go forwards
        doGrabGlyph(true);
        encoderDrive(DRIVE_SPEED, 4, 4, 2.0); // go forwards
        encoderDrive(FAST_SPEED, -6, -6,  2.0); // go forwards
        encoderLift(0.6, 10, 1.5);              // lower lift          
        doGrabGlyph(false);
        encoderDrive(FAST_SPEED, 7, 7, 2.0); // go forwards   
        encoderDrive(FAST_SPEED, -6, -6, 2.0); // go forwards          
    }
    
    void red_get_off_beam(int direction) {
        if (direction == 0) {
            encoderDrive(SLOW_SPEED, 3, 3, 1.5); // go forwards to knock jewel off        
            encoderDrive(SLOW_SPEED, 14, 14, 5.0); // get off beam;    
        } else {
            encoderDrive(SLOW_SPEED, -1.5, -1.5, 1.5); // go backwards to knock jewel off
            encoderDrive(SLOW_SPEED, 3, 3, 1.5); // go backwards to knock jewel off  
            encoderDrive(SLOW_SPEED, 15, 15, 5.0); // get off beam;            
        }
    }
    
    void red_1_forward() {
        double bd = 13;
        encoderDrive(DRIVE_SPEED, bd, bd, 2.0); // go forwards  
        gyroTurn(TURN_SPEED, -90 );
        gyroTurn(TURN_SPEED, -90 );        
        encoderDrive(FAST_SPEED, 5, 5, 1.0); // go forwards
        doGrabGlyph(true);
        encoderDrive(FAST_SPEED, 3, 3, 1.0); // go forwards
        encoderDrive(FAST_SPEED, -5, -5, 1.5); // go backwards
        encoderLift(0.6, 0, 1.5);              // lower lift           
        doGrabGlyph(false);
        encoderDrive(FAST_SPEED, 6, 6, 2.0); // go forwards   
        encoderDrive(FAST_SPEED, -4, -4, 2.0);         
        //asyncEncoderLift(0.6, 0, 1.5);
        // second glyph
            encoderDrive(FAST_SPEED, -6, -6, 2.0); // backwards
            //initGyro();
            resetAngle();
            gyroTurn(TURN_SPEED, -90);
            resetAngle();
            gyroTurn(TURN_SPEED, -90);
            //encoderLift(0.7, 0, 1.5);   
            doGrabGlyph(true);
            encoderDrive(0.9, 21, 21, 5.0); // try to grab a block
            resetAngle();
            gyroTurn(TURN_SPEED, -10);  
            doGrabGlyph(false);
            asyncEncoderLift(0.8, 700, 2);
            resetAngle();
            gyroTurn(TURN_SPEED, -170);
            gyroTurn(TURN_SPEED, -170);            
            encoderDrive(0.9, 28, 28, 4.0);
            encoderDrive(DRIVE_SPEED, 6, 6, 1.0);
            doGrabGlyph(true);
            encoderDrive(FAST_SPEED, -4, -4, 2.0); 
            encoderLift(0.7, 0, 2);
        /*
        encoderDrive(FAST_SPEED, 6, 6, 2.0); // go forwards   
        encoderDrive(FAST_SPEED, -5, -5, 2.0); // go forwards  
        */
    }
    
    void red_1_forward_internal(boolean grabSecond, double bd, int direction) {
        //bd = 4;
        //encoderDrive(SLOW_SPEED, 20, 20, 8.0); // go forwards
        if (direction == 0)
            encoderDrive(0.4, 13, 13, 2.5);
        else
            encoderDrive(SLOW_SPEED, 20, 20, 8.0);     
        encoderDrive(DRIVE_SPEED, -4, -4, 0.5); // backup to straighten up  
        encoderDrive(FAST_SPEED, bd, bd, 2.0); // go forwards  
        gyroTurn(TURN_SPEED, -90 );
        gyroTurn(TURN_SPEED, -90 );        
        encoderDrive(FAST_SPEED, 5, 5, 1.0); // go forwards
        doGrabGlyph(true);
        encoderDrive(DRIVE_SPEED, 3, 3, 1.0); // go forwards
        if (!grabSecond) {
            encoderDrive(FAST_SPEED, -6, -6, 1.5); // go backwards 
            encoderLift(0.6, 0, 1.5);              // lower lift        
            doGrabGlyph(false);
            encoderDrive(FAST_SPEED, 7, 7, 2.0); // go forwards   
        }
        // Code to grab another block
        if (grabSecond) {
            encoderDrive(FAST_SPEED, -5, -5, 1); // go backwards 
            encoderLift(0.7, 0, 1.5);              // lower lift        
            doGrabGlyph(false);
            encoderDrive(FAST_SPEED, 7, 7, 2.0); // go forwards               
            encoderDrive(0.9, -10, -10, 2.0); // backwards
            //doGrabGlyph(false);
            //encoderLift(0.7, 0, 1); 
            //initGyro();
            resetAngle();
            gyroTurn(TURN_SPEED, -90);
            resetAngle();
            gyroTurn(TURN_SPEED, -90);
            //encoderLift(0.7, 0, 1.5);   
            doGrabGlyph(true);
            encoderDrive(0.9, 21, 21, 2.0); // try to grab a block
            resetAngle();
            //gyroTurn(TURN_SPEED, -10);  
            doGrabGlyph(false);
            asyncEncoderLift(0.8, 700, 2);
            resetAngle();
            gyroTurn(TURN_SPEED, -180);
            gyroTurn(TURN_SPEED, -180);            
            encoderDrive(0.9, 28, 28, 3.0);
            encoderDrive(DRIVE_SPEED, 6, 6, 1.0);
            doGrabGlyph(true);
            encoderDrive(FAST_SPEED, -4, -4, 2.0); 
            encoderLift(0.7, 0, 2);
        } else {
            encoderDrive(FAST_SPEED, -5, -5, 2.0); // go forwards  
        }
    }

    
    public void doGrabGlyph(boolean topClose){
        double clawOffset = 0;        
        if (topClose) {
            clawOffset = -0.1;
        } else {
            clawOffset = 0.1;
        }
        robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
        robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);         
    }
    
    public synchronized void asyncEncoderLift(final double speed, final int count, final double timeout) {
        Runnable liftRunnable = new Runnable() {
            public void run() {
                encoderLift(speed, count, timeout);            
            }
        };
        Thread thread = new Thread(liftRunnable);
        thread.start();
    }
    
    public void encoderLift(double speed,
                         int count,
                         double timeoutS) {
    
        ElapsedTime liftRuntime = new ElapsedTime();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.liftMotor.setTargetPosition(count);

            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            liftRuntime.reset();
            robot.liftMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (liftRuntime.seconds() < timeoutS) &&
                   robot.liftMotor.isBusy()) {

                // Display it for the driver.
                //telemetry.addData("lift Target",  "Running to %7d", count);
                //telemetry.addData("liftPosition ",  "Running at %7d",
                //                            robot.liftMotor.getCurrentPosition());
                //telemetry.update();
            }

            // Stop all motion;
            robot.liftMotor.setPower(0);          
        }
    }
    
   public void relicArmExtend(double speed,
                         int count,
                         double timeoutS) {
        double desiredSpeed;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            /*int curPosition = robot.motorRelicLift.getCurrentPositio();
            if (curPosition > count) {
                desiredSpeed = - speed;
            }*/
            robot.motorRelicLift.setTargetPosition(count);

            // Turn On RUN_TO_POSITION
            robot.motorRelicLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorRelicLift.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   robot.motorRelicLift.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Relic lift Target",  "Running to %7d", count);
                telemetry.addData("Relic lift ",  "Running at %7d",
                                            robot.liftMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorRelicLift.setPower(0);          
        }
    }    
    
    public void doLift(int count) {
        encoderLift(0.7, count, 2);
    }
    
    public void doLift2() {
        robot.liftMotor.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.liftMotor.setPower(0.0);
    }
    
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches1, double rightInches1,
                             double timeoutS) {
        int newLeftTarget1;
        int newLeftTarget2;
        int newRightTarget1;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget1 = robot.motorLeftFront.getCurrentPosition() + (int)(leftInches1 * COUNTS_PER_INCH);
            newLeftTarget2 = robot.motorLeftBack.getCurrentPosition() + (int)(leftInches1 * COUNTS_PER_INCH);
            newRightTarget1 = robot.motorRightFront.getCurrentPosition() + (int)(rightInches1 * COUNTS_PER_INCH);
            newRightTarget2 = robot.motorRightBack.getCurrentPosition() + (int)(rightInches1 * COUNTS_PER_INCH);
            
            robot.motorLeftFront.setTargetPosition(newLeftTarget1);
            robot.motorLeftBack.setTargetPosition(newLeftTarget2);
            robot.motorRightFront.setTargetPosition(newRightTarget1);
            robot.motorRightBack.setTargetPosition(newRightTarget2);

            // Turn On RUN_TO_POSITION
            robot.motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                   (robot.motorLeftFront.isBusy() && robot.motorRightFront.isBusy()
                   && robot.motorLeftBack.isBusy() && robot.motorRightBack.isBusy()
                   )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d %7d, %7d %7d", newLeftTarget1,  newLeftTarget2, newRightTarget1, newRightTarget2);
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
            robot.motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move
        }
    }
}
