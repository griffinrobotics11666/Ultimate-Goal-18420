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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test Drive Mode", group="Iterative Opmode")
@Disabled
public class TestDriveMode extends OpMode {
    HardwareRobot robot = new HardwareRobot();

    private ElapsedTime runtime = new ElapsedTime();

    boolean debug = false;

    //SETS MAX AND MIN POSITIONS FOR SERVOS
    private static final double RIGHT_MAX_POS     =  0.67;
    private static final double RIGHT_MIN_POS     =  0.17;
    private static final double LEFT_MIN_POS     =  0.34;
    private static final double LEFT_MAX_POS     =  0.84;

    private static final double INCREMENT   = 0.003;// amount to increase servo
    private static final double DEBUG_INCREMENT = 0.0005; //amt to increase servo in debug (slower)

    private double  rightServoPosition = 0.5; // Start at halfway position
    private double  leftServoPosition = 0.5; // Start at halfway position

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double rearLeftPower;
        double rearRightPower;
        double frontLeftPower;
        double frontRightPower;

        if(!debug) {    //runs until debug is true

            if (gamepad1.dpad_right){   //opens the servo arm
                rightServoPosition += INCREMENT;
                leftServoPosition -= INCREMENT;
                if (rightServoPosition >= RIGHT_MAX_POS || leftServoPosition <= LEFT_MIN_POS ) {
                    rightServoPosition = RIGHT_MAX_POS;
                    leftServoPosition = LEFT_MIN_POS;
                }
                robot.rightArmServo.setPosition(rightServoPosition);
                robot.leftArmServo.setPosition(leftServoPosition);
            }

            if (gamepad1.dpad_left) {   //closes the servo arm
                rightServoPosition -= INCREMENT;
                leftServoPosition += INCREMENT;
                if (rightServoPosition <= RIGHT_MIN_POS || leftServoPosition >= LEFT_MAX_POS) {
                    rightServoPosition = RIGHT_MIN_POS;
                    leftServoPosition = LEFT_MAX_POS;
                }
                robot.rightArmServo.setPosition(rightServoPosition);
                robot.leftArmServo.setPosition(leftServoPosition);
            }

            if(gamepad1.dpad_up) {  //will run the debug statement until false
                debug = true;
            }

            //left stick
            double drive  =  gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            //right stick
            double turn = -gamepad1.right_stick_x;

            //calculates power
            rearLeftPower    = Range.clip(drive - strafe + turn, -1, 1) ;
            rearRightPower   = Range.clip(drive + strafe - turn, -1, 1) ;

            frontLeftPower = Range.clip(drive + strafe + turn, -1, 1) ;
            frontRightPower = Range.clip(drive - strafe - turn, -1, 1) ;

            // Send calculated power to rear wheels
            robot.rearLeftDrive.setPower(rearLeftPower);
            robot.rearRightDrive.setPower(rearRightPower);
            robot.frontLeftDrive.setPower(frontLeftPower);
            robot.frontRightDrive.setPower(frontRightPower);

            telemetry.addData("color_sensor_red", robot.colorSensor.red());
            telemetry.addData("color_sensor_green", robot.colorSensor.green());
            telemetry.addData("color_sensor_blue", robot.colorSensor.blue());
            telemetry.addData("distance_sensor_cm", robot.distanceSensor.getDistance(DistanceUnit.CM));


            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update(); //TODO do you need this?



        } else {    //runs if debug = true
            //telemetry.addData("Debug mode enabled."); //TODO

            if(gamepad1.b) {    //increases left servo slightly
                leftServoPosition += DEBUG_INCREMENT;
                if(leftServoPosition >= LEFT_MAX_POS) {
                    leftServoPosition = LEFT_MAX_POS;
                }
                robot.leftArmServo.setPosition(leftServoPosition);
            }

            if(gamepad1.b) {    //decreases left servo slightly
                leftServoPosition -= DEBUG_INCREMENT;
                if(leftServoPosition <= LEFT_MIN_POS) {
                    leftServoPosition = LEFT_MIN_POS;
                }
                robot.leftArmServo.setPosition(leftServoPosition);
            }
            if(gamepad1.y) {    //increases right servo slightly
                rightServoPosition += DEBUG_INCREMENT;
                if(rightServoPosition >= RIGHT_MAX_POS) {
                    rightServoPosition = RIGHT_MAX_POS;
                }
                robot.rightArmServo.setPosition(rightServoPosition);
            }
            if(gamepad1.x) {    //decreases right servo slightly
                rightServoPosition -= DEBUG_INCREMENT;
                if(rightServoPosition <= RIGHT_MIN_POS) {
                    rightServoPosition = RIGHT_MIN_POS;
                }
                robot.rightArmServo.setPosition(rightServoPosition);
            }


            if(gamepad1.dpad_down) {    //will exit out of loop if down is pressed
                debug = false;
            }

            //adds telemetry data for servos to phone
            telemetry.addData("Left servo Position", "%5.2f", leftServoPosition);
            telemetry.addData("Right servo Position", "%5.2f", rightServoPosition);
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}