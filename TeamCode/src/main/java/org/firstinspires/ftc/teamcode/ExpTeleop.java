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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Nalu Teleop 2", group="Pushbot")
//@Disabled
public class ExpTeleop extends OpMode{

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.RobotHardware robot       = new org.firstinspires.ftc.teamcode.RobotHardware(false);
    /*
     * Code to run ONCE when the driver hits INIT
     */

    private final double THRESHOLD = 0.05;
    private final double SLOWDOWN_FACTOR = 3.0;
    private final double SHOOT_TIME = 2.0;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.addData("Say", "Nalu Zou");
        // Send telemetry message to signify robot running;
        // *******************************************
        telemetry.addData("Welcome to the experimental robot", "");
        // *******************************************
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        double MAX_FWD      =  1.0;     // Maximum FWD power applied to motor
        double MAX_REV      = -1.0;     // Maximum REV power applied to motor
        double leftPower    = 0;
        double rightPower   = 0;
    }

    private void move(double left, double right) {
        robot.leftFrontMotor.setPower(left);
        robot.leftBackMotor.setPower(left);
        robot.rightFrontMotor.setPower(right);
        robot.rightBackMotor.setPower(right);

        //telemetry.addData("driver", left+ "    " + right);
    }
    /*
    private void vertical(boolean dir) {

    }
    private void horizontal (boolean dir) {

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //double leftStick;
        //double rightStick;
        //leftStick   = -gamepad1.left_stick_y;
        //rightStick  = -gamepad1.right_stick_y;

        // Use gamepad left & right Bumpers to open and close the claw
        //if (gamepad1.right_bumper)
        //    clawOffset += CLAW_SPEED;
        //else if (gamepad1.left_bumper)
        //    clawOffset -= CLAW_SPEED;

        // Use gamepad buttons to drive up, down, left, or right
        //press a to launch beacon0
        telemetry.addData("Motor Power: ", "shooter: " + robot.shooter.getPower() +
                                            " collector: " + robot.collector.getPower() +
                                            " left front: " + robot.leftFrontMotor.getPower() +
                                            " left back: " + robot.leftBackMotor.getPower() +
                                            " right front: " + robot.rightFrontMotor.getPower() +
                                            " right back: " + robot.rightBackMotor.getPower());
        double y = gamepad1.left_stick_y;
        double x = gamepad1.right_stick_x;


        x = (Math.abs(x) < THRESHOLD) ? 0 : x;
        y = (Math.abs(y) < THRESHOLD) ? 0 : y;

        double shooterPower = gamepad1.right_trigger;
        double collectorPower = gamepad1.left_trigger;
        long rotateTime = 500;
        //Thread.sleep(rotateTime);
        //double shooterPower = 0;
        //double collectorPower = 0;

        robot.shooter.setPower(shooterPower);
        if (gamepad1.left_bumper)
        {
            robot.collector.setPower(-1.0);
        }
        else
        {
            robot.collector.setPower(collectorPower);
        }

        if (gamepad1.b) {
            x /= SLOWDOWN_FACTOR;
            y /= SLOWDOWN_FACTOR;
            telemetry.addData("Driver", "slowed down");
        }

        telemetry.addData("Controller: ", "b: " + gamepad1.b + " right trigger: " + gamepad1.right_trigger + " left trigger: " + gamepad1.left_trigger + " x: " + x + " y: " + y);
        if (x != 0) {
            if (y > 0) {
                move(y - x, y + x);
                telemetry.addData("Driver", "fwd + turn");
            }
            else if (y < 0) {
                move(y + x, y - x);
                telemetry.addData("Driver", "bwd + turn");
            }
            else {
                 telemetry.addData("Driver", "turning");
                 move(-x, x);
            }
        }
        else {
            telemetry.addData("Driver", "moving");
            move(y, y);
        }

        //telemetry.addData("right", "%.2f", right);
        updateTelemetry(telemetry);
    }


    private void shoot() {
        long startTime = System.nanoTime();
        while(System.nanoTime()- startTime < SHOOT_TIME) {
            robot.shooter.setPower(1.0);
        }
        robot.shooter.setPower(0);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}