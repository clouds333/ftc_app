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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.RobotCoreLynxUsbDevice;
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
    double          relicOffset = 0;
    double          relicPosition = 0;
    final double    RELIC_SPEED = 1.0;
    final double    LIFT_SPEED      =1.0;
    private ElapsedTime liftElapsedTime = new ElapsedTime();
    int             liftTimeout     = 0;
    double          relicClawOffset = 0;
    final double    RELIC_CLAW_SPEED = 0.02;
    double          relicArmOffset = 0;
    final double    RELIC_ARM_SPEED = 0.02;    
    private ElapsedTime runtime = new ElapsedTime();
    
    public void encoderLift(double speed,
                         int count,
                         double timeoutS) {
    
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.liftMotor.setTargetPosition(count);

            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   robot.liftMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("lift Target",  "Running to %7d", count);
                telemetry.addData("liftPosition ",  "Running at %7d",
                                            robot.liftMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.liftMotor.setPower(0);          
        }
    }
    
    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double relicExtend;
        
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
            
            left *= 0.7;
            right *= 0.7;

            // Output the safe vales to the motor drives.
            robot.motorLeftFront.setPower(left);
            robot.motorLeftBack.setPower(left);
            robot.motorRightFront.setPower(right);
            robot.motorRightBack.setPower(right);


            // use gamepad2 joystick to control relic extension
            relicExtend  =  gamepad2.left_stick_x;
            relicExtend = Range.clip(relicExtend, -0.3, 0.3);
            robot.motorRelicLift.setPower(relicExtend);
            
            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad2.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad2.left_bumper)
                clawOffset -= CLAW_SPEED;
               
            // Use gamepad dpad_left and dpad_right to open relic claw
            if (gamepad2.dpad_down)
                relicClawOffset += RELIC_CLAW_SPEED;
            else if (gamepad2.dpad_up)
                relicClawOffset -= RELIC_CLAW_SPEED;
            else if(gamepad2.dpad_left)
                relicArmOffset += RELIC_ARM_SPEED;
            else if(gamepad2.dpad_right)
                relicArmOffset -= RELIC_ARM_SPEED;
            
            else if (gamepad1.left_bumper)
                robot.backServo.setPosition(0.0);
            else if (gamepad1.right_bumper)
                robot.backServo.setPosition(1.0);
                
            else if (gamepad1.dpad_left)
                encoderLift(0.8, 300, 4);
            else if (gamepad1.dpad_right)
                encoderLift(0.8, 600, 4);                
            else if (gamepad1.dpad_up)
                encoderLift(0.8, 1300, 4);
            else if (gamepad1.dpad_down)
                encoderLift(0.8, 0, 4);
            /*    
            else if(gamepad2.dpad_up)
                robot.motorRelicLift.setPower(-0.25);
            else if(gamepad2.dpad_down)
                robot.motorRelicLift.setPower(0);
            else if(gamepad2.dpad_left)
                robot.motorRelicLift.setPower(0.25);*/

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.1, 0.1);
            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

            relicClawOffset = Range.clip(relicClawOffset, 0, 0.86);
            robot.relicClaw.setPosition(relicClawOffset);
            
            relicArmOffset = Range.clip(relicArmOffset, 0.00, 0.8);
            robot.relicArm.setPosition(relicArmOffset);
            
            // increase of decrease speed of motors
            // how to do this, damp the power by a factor

             //Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad2.y) {
                liftOffset += LIFT_SPEED;
            } else if (gamepad2.a) {
                liftOffset -= LIFT_SPEED;
            } else if (gamepad2.b) {
                liftOffset=0.0;
            }

            robot.liftMotor.setPower(liftOffset);
            liftOffset = Range.clip(liftOffset, -0.3, 0.3);
            robot.liftMotor.setPower(liftOffset);
            //robot.liftServo.setPosition(robot.MID_SERVO + liftOffset);
            
            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("relicClaw",  "Offset = %.2f", relicClawOffset);
            telemetry.addData("relicArm",  "Offset = %.2f", relicArmOffset);            
            telemetry.addData("liftOffset", "Offset=%.2f", liftOffset);
            telemetry.addData("liftPosition", "liftPosition=%d", robot.liftMotor.getCurrentPosition());
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
