package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;


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
public class RobotHardware {
    /* Public OpMode members. */
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;
    public Servo   pusher = null;
    public ColorSensor  sensor = null;
    public ColorSensor bottomSensor = null;
    public DcMotor collector = null;
    public DcMotor shooter = null;
    public ModernRoboticsI2cGyro gyro = null;
    public DcMotor lifter = null;
    //public Servo    leftClaw        = null;
    //public Servo    rightClaw       = null;
    //public Servo    backClaw        = null;
    //public Servo    frontClaw       = null;

    //public static final double MID_SERVO       =  0.5 ;

    public static final double FORWARD_POWER = 0.45;
    public static final double BACKWARD_POWER = -0.45;
    public static final double TURN_POWER = 0.2;
    public static final double BACKWARD_TURN_POWER = -0.2;
    public static final double STOP_POWER = 0.0;

    private boolean mode = false; //false = teleop true = autonomous

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    private boolean direction; //false = backwards
    /* Constructor */
    public RobotHardware(boolean newMode){
        mode = newMode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor   = hwMap.dcMotor.get("lf motor");
        rightFrontMotor  = hwMap.dcMotor.get("rf motor");
        leftBackMotor    = hwMap.dcMotor.get("lb motor");
        rightBackMotor   = hwMap.dcMotor.get("rb motor");
        pusher           = hwMap.servo.get("pusher");
        sensor           = hwMap.colorSensor.get("sensor");
        bottomSensor     = hwMap.colorSensor.get("bottom sensor");
        collector        = hwMap.dcMotor.get("collector");
        shooter          = hwMap.dcMotor.get("shooter");
        gyro             = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");
        lifter           = hwMap.dcMotor.get("lifter");

        bottomSensor.setI2cAddress(I2cAddr.create8bit(0x10));
        sensor.setI2cAddress(I2cAddr.create8bit(0x16));
        gyro.setI2cAddress(I2cAddr.create8bit(0x30));
        sensor.enableLed(false);

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        collector.setDirection (DcMotor.Direction.FORWARD);
        shooter.setDirection (DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        collector.setPower(0);
        shooter.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        if (mode) {
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        // leftClaw = hwMap.servo.get("left claw");
        // rightClaw = hwMap.servo.get("right claw");
        // backClaw = hwMap.servo.get("back claw");
        // frontClaw = hwMap.servo.get("front claw");
        // leftClaw.setPosition(MID_SERVO);
        // rightClaw.setPosition(MID_SERVO);
        // backClaw.setPosition(MID_SERVO);
        // frontClaw.setPosition(MID_SERVO);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    static final int     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    public void shoot() {
        shooter.setMode(DcMotor.RunMode.RESET_ENCODERS);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int currentPosition = shooter.getCurrentPosition();
        int newTarget = currentPosition + 1680;
        shooter.setTargetPosition(newTarget);
        //telemetry.addData("Shoot",  "current=%7d: target=%7d",      currentPosition,  newTarget);
        //telemetry.update()'
        shooter.setPower(1);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (shooter.isBusy())
        {

        }
        shooter.setPower(0);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}