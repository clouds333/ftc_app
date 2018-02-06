package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareVoyagerbot
{
    /* Public OpMode members. */
    //public DcMotor  leftMotor   = null;
    //public DcMotor  rightMotor  = null;
    //public DcMotor  armMotor    = null;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;
    //public Servo    lowerLeftClaw    = null;
    //public Servo    lowerRightClaw   = null;    
    public Servo    colorServo  = null;
    public Servo    backServo   = null;
    public Servo    relicArm    = null;
    public Servo    relicClaw   = null;

    public DcMotor  motorLeftFront   = null;
    public DcMotor  motorLeftBack  = null;
    public DcMotor  motorRightFront   = null;
    public DcMotor  motorRightBack  = null;
    public DcMotor  motorRelicLift = null;
    public DcMotor  motorLeftIntake = null;
    public DcMotor  motorRightIntake = null;
    public DcMotor  liftMotor = null;
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.7 ;
    public static final double ARM_DOWN_POWER  = -0.7 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareVoyagerbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean setServo) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeftBack   = hwMap.dcMotor.get("m2");   //has encoder
        motorLeftFront  = hwMap.dcMotor.get("m3");
        motorRightBack   = hwMap.dcMotor.get("m0");  //has encoder
        motorRightFront  = hwMap.dcMotor.get("m1");
        motorRelicLift   = hwMap.dcMotor.get("relicLift"); //has encoder
        liftMotor        = hwMap.dcMotor.get("liftMotor"); //has encoder
    
        //armMotor    = hwMap.dcMotor.get("left_arm");
        motorLeftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorLeftBack.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //motorRelicLift.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        
        // Set all motors to zero power
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorRelicLift.setPower(0);
        //motorLeftIntake.setPower(0);
        //motorRightIntake.setPower(0);
        liftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRelicLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Define and initialize ALL installed servos.
        leftClaw = hwMap.servo.get("leftArm");
        rightClaw = hwMap.servo.get("rightArm");
        //lowerLeftClaw = hwMap.servo.get("lowerLeftClaw");
        //lowerRightClaw = hwMap.servo.get("lowerRightClaw");
        colorServo = hwMap.servo.get("colorServo");
        //liftServo = hwMap.servo.get("liftServo"); changed to motor
        backServo = hwMap.servo.get("backServo");
        relicArm = hwMap.servo.get("relicArm");
        relicClaw = hwMap.servo.get("relicClaw");
        
        if (setServo) {
            //leftClaw.setPosition(MID_SERVO);
            //rightClaw.setPosition(MID_SERVO);
            //lowerLeftClaw.setPosition(MID_SERVO);
            //lowerRightClaw.setPosition(MID_SERVO);
            colorServo.setPosition(0.0);
            //backServo.setPosition(1.0);
            //relicArm.setPosition(0.0);
            //relicClaw.setPosition(MID_SERVO);
        }

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

