/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Timer;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

@Autonomous(name="Exp Autonomous", group="Pushbot")
//@Disabled
public class ExpAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.ExpHardware robot = new org.firstinspires.ftc.teamcode.ExpHardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private int state = 0; //0 start moving forward      1 moving and stopping when reached target
    private final int finalState = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("driver", "starting");
        robot.init(hardwareMap);

        while(state <= finalState) {
            telemetry.addData("state", state);
            switch (state) {
                case 0:
                    telemetry.addData("driver", "running loop");
                    driveForwardDistance(1.0, 5);
                    state++;
                    break;
                case 1:
                    if (!isBusy())
                    {
                        state++;
                    }
                    break;
            }
        }
        /* reset values */
        state = 0;
        telemetry.update();
    }

    /*private void driveForwardDistance2(double power, int distance)
    {
        if (robot != null)
        {
            telemetry.addData("driver", "robot not null");
        }
        else {
            telemetry.addData("driver", "robot is null");
        }
        if (robot.leftFrontMotor != null)
        {
            telemetry.addData("driver", "motor not null");
        }
        else {
            telemetry.addData("driver", "motor is null 2");
        }
        telemetry.addData("driver", "current run mode: " + robot.leftFrontMotor.getMode());

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontMotor.setTargetPosition((int)(COUNTS_PER_INCH * distance));
        robot.leftBackMotor.setTargetPosition((int)(COUNTS_PER_INCH * distance));
        robot.rightFrontMotor.setTargetPosition((int)(COUNTS_PER_INCH * distance));
        robot.rightBackMotor.setTargetPosition((int)(COUNTS_PER_INCH * distance));

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontMotor.setPower(-1.0);
        robot.leftBackMotor.setPower(-1.0);
        robot.rightFrontMotor.setPower(1.0);
        robot.rightBackMotor.setPower(1.0);

        while (robot.leftFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.rightBackMotor.isBusy())
        {
            telemetry.addData("driver", "busy");
            return;
        }
        telemetry.addData("driver", "called");

        robot.leftFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    */

    private boolean isBusy()
    {
        return (robot.leftFrontMotor.isBusy() &&
                robot.leftBackMotor.isBusy() &&
                robot.rightFrontMotor.isBusy() &&
                robot.rightBackMotor.isBusy());
    }
    private boolean driveForwardDistance(double power, int distance)
    {
        /*if it is still running */
        if (isBusy())
        {
            telemetry.addData("motors", "busy");
            return true;
        }
        robot.leftFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        /*if (robot != null)
        {
            telemetry.addData("driver", "robot not null");
        }
        else {
            telemetry.addData("driver", "robot is null");
        }
        if (robot.leftFrontMotor != null)
        {
            telemetry.addData("driver", "motor not null");
            telemetry.update();
        }
        else {
            telemetry.addData("driver", "motor is null 2");
            telemetry.update();
        }
        telemetry.addData("driver", "current run mode: " + robot.leftFrontMotor.getMode());
        telemetry.update();*/
        telemetry.addData("driver", "running");
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontMotor.setTargetPosition((int)(COUNTS_PER_INCH * distance));
        robot.leftBackMotor.setTargetPosition((int)(COUNTS_PER_INCH * distance));
        robot.rightFrontMotor.setTargetPosition((int)(COUNTS_PER_INCH * distance));
        robot.rightBackMotor.setTargetPosition((int)(COUNTS_PER_INCH * distance));

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontMotor.setPower(1.0);
        robot.leftBackMotor.setPower(1.0);
        robot.rightFrontMotor.setPower(1.0);
        robot.rightBackMotor.setPower(1.0);

        telemetry.addData("driver", "called");

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return false;
    }
}