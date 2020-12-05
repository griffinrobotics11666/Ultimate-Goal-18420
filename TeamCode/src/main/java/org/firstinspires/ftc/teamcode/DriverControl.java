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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;



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

@TeleOp(name="Driver Control", group="Iterative Opmode")
//@Disabled
public class DriverControl extends OpMode {
    HardwareRobot robot = new HardwareRobot();


    private ElapsedTime runtime = new ElapsedTime();

    //set up variables for the button presses
    boolean debug = false, debugChanged = false;
    boolean inPickupPos = true, isXchanged;
    boolean inLaunchPos = false, isBchanged;
    boolean isOpen = true, isOpenChanged = false;
    boolean isRoofRaised = true, isYchanged= false; // true: arm is up, false: arm is down
    boolean shootyIsRunning = false, shootyIsRunningChanged = false;
    boolean isShooting = false, shootyIsShootingChanged = false;

    // Setup a variable for each drive wheel to save power level for telemetry
    double rearLeftPower;
    double rearRightPower;
    double frontLeftPower;
    double frontRightPower;

    //SETS MAX AND MIN POSITIONS FOR SERVOS
    private static final double CLAW_SERVO_OPEN_POS     =  0.35;
    private static final double CLAW_SERVO_CLOSE_POS     =  0.55;

    private static final double SHOOTY_ROTATION_LAUNCH_POS     =  0;//todo
    private static final double SHOOTY_ROTATION_FLAT_POS     =  0.64;

    private static final double CLAW_ROTATION_SERVO_PICKUP     =  0.35;
    private static final double CLAW_ROTATION_SERVO_DROP     =  0.49;

    private static final double SHOOTY_BOI_SERVO_SHOOT_POS     =  0.64;
    private static final double SHOOTY_BOI_SERVO_LOAD_POS     =  0.47;

    private static final double DEBUG_INCREMENT = 0.0005; //amt to increase servo in debug (slower)


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        //set DC Motor drive modes
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.shootyMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //set arm Motor to run with encoder
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.shootyBoi.setPosition(SHOOTY_BOI_SERVO_LOAD_POS);
        robot.clawRotationServo.setPosition(CLAW_ROTATION_SERVO_PICKUP);
        robot.clawServo.setPosition(CLAW_SERVO_OPEN_POS);
        robot.shootyRotaion.setPosition(SHOOTY_ROTATION_FLAT_POS);

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

        if(!debug) {    //runs until debug is true

            if(gamepad1.back && !debugChanged){   //toggles debug
                debug = true;
                debugChanged = true;
            } else if (!gamepad1.back) {
                debugChanged = false;
            }

            if(gamepad1.x && !isXchanged){   //toggles pickup and drop position for rotation
                robot.clawRotationServo.setPosition(inPickupPos ? CLAW_ROTATION_SERVO_DROP : CLAW_ROTATION_SERVO_PICKUP);
                inPickupPos = !inPickupPos;
                isXchanged = true;
            } else if (!gamepad1.x) {
                isXchanged = false;
            }

            if(gamepad1.a && !isOpenChanged){   //toggles open and close of claw servo
                robot.clawServo.setPosition(isOpen ? CLAW_SERVO_CLOSE_POS : CLAW_SERVO_OPEN_POS);
                isOpen = !isOpen;
                isOpenChanged = true;
            } else if (!gamepad1.a) {
                isOpenChanged = false;
            }

            if(gamepad1.dpad_left) {
                robot.shootyRotaion.setPosition(robot.shootyRotaion.getPosition() - 0.01);
                if(robot.shootyRotaion.getPosition() < 0){
                    robot.shootyRotaion.setPosition(0);
                }
            } else if(gamepad1.dpad_right) {
                robot.shootyRotaion.setPosition(robot.shootyRotaion.getPosition() + 0.01);
                if(robot.shootyRotaion.getPosition() > 1){
                    robot.shootyRotaion.setPosition(1);
                }
            }
            telemetry.addData("Shooty Rotation servo Position: ", "%5.2f", robot.shootyRotaion.getPosition());

//            if(gamepad1.b && !isBchanged){   //toggles position of shooty rotation servo
//                robot.shootyRotaion.setPosition(inLaunchPos ? SHOOTY_ROTATION_FLAT_POS : SHOOTY_ROTATION_LAUNCH_POS);
//                inLaunchPos = !inLaunchPos;
//                isBchanged = true;
//            } else if (!gamepad1.b) {
//                isBchanged = false;
//            }

            if(gamepad1.y && !isYchanged){   //changes value of isRoofRaised
                isYchanged = true;
                if(isRoofRaised){   //MOVE ARM MOTOR DOWN
                    if(!robot.touchyKid.getState()) {   //checks if limit switch is closed
                        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.armMotor.setTargetPosition(5562);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(1);
                        if (robot.armMotor.getCurrentPosition() >= 5562) {
                            robot.armMotor.setPower(0.0);
                        }
                    }
                } else if(!isRoofRaised){   //MOVE ARM MOTOR UP
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setTargetPosition(0);
                    robot.armMotor.setPower(1);
                    if(!robot.touchyKid.getState()){
                        robot.armMotor.setPower(0.0);
                    }
                }
                isRoofRaised = !isRoofRaised;
            } else if (!gamepad1.y) {
                isYchanged = false;
            }


            if(gamepad1.right_bumper && !shootyIsShootingChanged){   //toggles open and close of claw servo
                robot.shootyBoi.setPosition(SHOOTY_BOI_SERVO_SHOOT_POS);
                double pressTime = runtime.milliseconds();
                while(runtime.milliseconds()-pressTime < 250){  //wait until the shooty servo has fully moves forward.
                    telemetry.addData("Wait " ,"true");
                    telemetry.update();
                }
                robot.shootyBoi.setPosition(SHOOTY_BOI_SERVO_LOAD_POS);
                isShooting = !isShooting;
                shootyIsShootingChanged = true;
            } else if (!gamepad1.right_bumper) {
                shootyIsShootingChanged = false;
            }

            if(gamepad1.start && !shootyIsRunningChanged){   //toggles turning on and off shooty motor
                robot.shootyMotor.setPower(shootyIsRunning ? 0 : 0.75);
                shootyIsRunning = !shootyIsRunning;
                shootyIsRunningChanged = true;
            } else if (!gamepad1.start) {
                shootyIsRunningChanged = false;
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

//            telemetry.addData("color_sensor_red", robot.colorSensor.red());
//            telemetry.addData("color_sensor_green", robot.colorSensor.green());
//            telemetry.addData("color_sensor_blue", robot.colorSensor.blue());
//            telemetry.addData("distance_sensor_cm", robot.distanceSensor.getDistance(DistanceUnit.CM));


            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Is Open", ": " + isOpen);
            telemetry.addData("Is roof raised", ": " + isRoofRaised);
            telemetry.addData("Is arm motor busy", ""  + robot.armMotor.isBusy());
            telemetry.addData("Arm Motor Encoder Position", robot.armMotor.getCurrentPosition());
            telemetry.addData("Is Touching sensor", ": " + !robot.touchyKid.getState());

            telemetry.update();



        } else {    //runs if debug = true
            telemetry.addData("Debug mode: " ,"enabled.");

            //claw servo
            if(gamepad1.b) {
                robot.clawServo.setPosition(robot.clawServo.getPosition() + DEBUG_INCREMENT);
            } else if(gamepad1.a) {
                robot.clawServo.setPosition(robot.clawServo.getPosition() - DEBUG_INCREMENT);
            }
            telemetry.addData("Claw servo Position: ", "%5.2f", robot.clawServo.getPosition());

            //rotation servo
            if(gamepad1.y) {
                robot.clawRotationServo.setPosition(robot.clawRotationServo.getPosition() + DEBUG_INCREMENT);
            } else if(gamepad1.x) {
                robot.clawRotationServo.setPosition(robot.clawRotationServo.getPosition() - DEBUG_INCREMENT);
            }
            telemetry.addData("Claw Rotation servo Position: ", "%5.2f", robot.clawRotationServo.getPosition());

            //shooty boi servo
//            if(gamepad1.right_bumper) {
//                robot.shootyBoi.setPosition(robot.shootyBoi.getPosition() + DEBUG_INCREMENT);
//            } else if (gamepad1.left_bumper) {
//                robot.shootyBoi.setPosition(robot.shootyBoi.getPosition() - DEBUG_INCREMENT);
//            }
//            telemetry.addData("Shooty Boi servo Position: ", "%5.2f", robot.shootyBoi.getPosition());


            //shooty rotation servo
            if(gamepad1.dpad_left) {
                robot.shootyRotaion.setPosition(robot.shootyRotaion.getPosition() - DEBUG_INCREMENT);
                if(robot.shootyRotaion.getPosition() < 0){
                    robot.shootyRotaion.setPosition(0);
                }
            } else if(gamepad1.dpad_right) {
                robot.shootyRotaion.setPosition(robot.shootyRotaion.getPosition() + DEBUG_INCREMENT);
                if(robot.shootyRotaion.getPosition() > 1){
                    robot.shootyRotaion.setPosition(1);
                }
            }
            telemetry.addData("Shooty Rotation servo Position: ", "%5.2f", robot.shootyRotaion.getPosition());

            //arm Motor
            if(gamepad1.dpad_up){
                robot.armMotor.setPower(-.5);
            }
            else if (gamepad1.dpad_down){
                robot.armMotor.setPower(.5);
            }
            else{
                robot.armMotor.setPower(0);
            }
            telemetry.addData("Arm Motor ", "Position: %7d", robot.armMotor.getCurrentPosition());
            telemetry.addData("touchyKid", robot.touchyKid.getState());

            if(gamepad1.start && !shootyIsRunningChanged){   //toggles turning on and off shooty motor
                robot.shootyMotor.setPower(shootyIsRunning ? 0 : 0.75);
                shootyIsRunning = !shootyIsRunning;
                shootyIsRunningChanged = true;
            } else if (!gamepad1.start) {
                shootyIsRunningChanged = false;
            }

            if(gamepad1.back && !debugChanged){   //toggles debug
                debug = false;
                debugChanged = true;
            } else if (!gamepad1.back) {
                debugChanged = false;
            }
            telemetry.update();
        }
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }



}