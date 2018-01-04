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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp public class VoyagerBotTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareVoyagerbot robot           = new HardwareVoyagerbot();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.05 ;                   // sets rate to move servo
    double          liftOffset      = 0;
    double          liftPosition   = 0;
    final double    LIFT_SPEED      =0.20;
    private ElapsedTime liftElapsedTime = new ElapsedTime();
    int             liftTimeout     = 0;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        boolean isLifting = false;
        int liftDirection = 0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // set the arm to normal position
        robot.colorServo.setPosition(0.2);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            robot.motorLeftFront.setPower(left);
            robot.motorLeftBack.setPower(left);
            robot.motorRightFront.setPower(right);
            robot.motorRightBack.setPower(right);

            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad2.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad2.left_bumper)
                clawOffset -= CLAW_SPEED;
                
            if (gamepad1.left_bumper)
                robot.backServo.setPosition(0.0);
            else if (gamepad1.right_bumper)
                robot.backServo.setPosition(1.0);

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.4, 0.4);
            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);


            // increase of decrease speed of motors
            // how to do this, damp the power by a factor

             //Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad2.y) {
                liftOffset += LIFT_SPEED;
            } else if (gamepad2.a) {
                liftOffset -= LIFT_SPEED;
            }
            else if (gamepad2.b) {
                liftOffset=0.0;
            }

            liftOffset = Range.clip(liftOffset, -0.5, 0.5);
            robot.liftServo.setPosition(robot.MID_SERVO + liftOffset);

            if (isLifting) {
                if (liftElapsedTime.milliseconds() > liftTimeout) {
                    robot.liftServo.setPosition(0.0);
                    isLifting = false;
                } else {
                    if (liftDirection == 0) {
                        robot.liftServo.setPosition(robot.MID_SERVO + 0.5);
                    } else {
                        robot.liftServo.setPosition(robot.MID_SERVO - 0.5);                        
                    }
                }
            }

            if (gamepad1.y) {
                robot.liftServo.setPosition(robot.MID_SERVO + 0.5);
                isLifting = true;
                liftDirection = 0;
                liftTimeout = 5500;
                liftElapsedTime.reset();
                //sleep(5500);
                //robot.liftServo.setPosition(0.0);
            } else if (gamepad1.a) {
                robot.liftServo.setPosition(robot.MID_SERVO -0.5);
                isLifting = true;
                liftDirection = 1;
                liftTimeout = 4700;
                liftElapsedTime.reset();
                //sleep(4700);
                //robot.liftServo.setPosition(0.0);
            } else if (gamepad1.x) {
                robot.liftServo.setPosition(robot.MID_SERVO+0.5);
                isLifting = true;
                liftTimeout = 1000;
                liftDirection = 0;
                liftElapsedTime.reset();
                //sleep(1000);
                //robot.liftServo.setPosition(0.0);
            } else if (gamepad1.b) {
                robot.liftServo.setPosition(robot.MID_SERVO-0.5);
                isLifting = true;
                liftTimeout = 1000;
                liftDirection = 1;
                liftElapsedTime.reset();
                //sleep(1000);
                //robot.liftServo.setPosition(0.0);
            }

            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("liftOffset", "Offset=%.2f", liftOffset);
            telemetry.addData("liftPosition", "liftPosition=%.2f", liftPosition);
            telemetry.addData("liftTime=%.2f",liftElapsedTime.milliseconds());
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
