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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="Autonomous Test", group="Linear Opmode")
//@Disabled
public class AutonomousTest extends LinearOpMode {
    HardwareRobot ned = new HardwareRobot();

    Orientation angleExpansion;
    Orientation angleControl;



    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // encoder counts per revolution
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {

        ned.init(hardwareMap);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                ned.rearRightDrive.getCurrentPosition(),
                ned.rearLeftDrive.getCurrentPosition(),
                ned.frontLeftDrive.getCurrentPosition(),
                ned.frontRightDrive.getCurrentPosition());

        telemetry.update();


        //telemetry.addData("Status", "Ready");
        //telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        ned.imuControl.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        ned.imuExpansion.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //timeout is a failsafe to stop all motors if it takes too long
        encoderDrive(DRIVE_SPEED, 24, 10.0);  // S1: Forward 48 Inches with 5 Sec timeout
        turn(-90, .5);
        sleep(500);
        turn(90, .5);
        encoderDrive(DRIVE_SPEED, -24, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        turn(90, .5);
        sleep(500);
        turn(90, .5);


        telemetry.addData("Path", "Complete");
        telemetry.update();

    }






    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void encoderDrive(double speed, double inches, double timeoutS) {
        int encoderTarget;


        double currentSpeed = 0.2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            //the motors will come to a hard stop instead of coasting
            ned.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ned.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ned.rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ned.rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            ned.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ned.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ned.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ned.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //Reset Encoders

            ned.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ned.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ned.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ned.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // calculate target positions
            encoderTarget = ned.frontLeftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            //set target positions for motors
            ned.frontLeftDrive.setTargetPosition(encoderTarget);
            ned.frontRightDrive.setTargetPosition(encoderTarget);
            ned.rearLeftDrive.setTargetPosition(encoderTarget);
            ned.rearRightDrive.setTargetPosition(encoderTarget);

            // Turn On RUN_TO_POSITION
            ned.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ned.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ned.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ned.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            ned.frontLeftDrive.setPower(currentSpeed);
            ned.frontRightDrive.setPower(currentSpeed);
            ned.rearLeftDrive.setPower(currentSpeed);
            ned.rearRightDrive.setPower(currentSpeed);


            //if one of these is false, the loop will exit and will continue to set power for all wheels to 0
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && ned.frontLeftDrive.isBusy() && ned.frontRightDrive.isBusy()
                    && ned.rearLeftDrive.isBusy() && ned.rearRightDrive.isBusy()) {

                if (currentSpeed < speed && !(encoderTarget - ned.frontLeftDrive.getCurrentPosition() < 0.25 * encoderTarget)) { //TODO create ramp up & ramp down methods edit: this done?
                    currentSpeed += 0.005;

                }

                if (encoderTarget - ned.frontLeftDrive.getCurrentPosition() < 0.25 * encoderTarget) {
                    //currentSpeed = speed * (newFrontLeftTarget - ned.frontLeftDrive.getCurrentPosition()) / (newFrontLeftTarget);
                    currentSpeed -= 0.005;
                }
                ned.frontLeftDrive.setPower(currentSpeed);
                ned.frontRightDrive.setPower(currentSpeed);
                ned.rearLeftDrive.setPower(currentSpeed);
                ned.rearRightDrive.setPower(currentSpeed);
                telemetry.addData("Current Speed", currentSpeed);
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", encoderTarget);
                telemetry.addData("Path2", "Running at %7d", ned.frontLeftDrive.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            ned.frontLeftDrive.setPower(0);
            ned.frontRightDrive.setPower(0);
            ned.rearLeftDrive.setPower(0);
            ned.rearRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            ned.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ned.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ned.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ned.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void turn(double turnAngle, double speed) {
        double currentAngle = readDoubleAngle();
        double lastAngle = currentAngle;
        double deltaAngle = 0;
        double totalAngle = 0;
        if (Math.abs(deltaAngle) >= 180) {
            deltaAngle = currentAngle + lastAngle;
        }
        double increment = .5;
        double power = 0;
        while (opModeIsActive() && Math.abs(totalAngle - turnAngle) > .1) {
            deltaAngle = currentAngle - lastAngle;
            totalAngle += deltaAngle;
            currentAngle = readDoubleAngle();
            power += increment;
            if (power > speed){
                power = speed;
            }
            else if (power < (-1)){
                power = -1;
            }
            ned.frontLeftDrive.setPower(power);
            ned.frontRightDrive.setPower(power);
            ned.rearLeftDrive.setPower(power);
            ned.rearRightDrive.setPower(power);
            if(Math.abs(totalAngle) > Math.abs(turnAngle)){
                increment = increment * (-0.5);
            }
        }
            ned.frontLeftDrive.setPower (0);
            ned.frontRightDrive.setPower(0);
            ned.rearLeftDrive.setPower(0);
            ned.rearRightDrive.setPower(0);

    }


    public String readAngle() {
        angleControl = ned.imuControl.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleExpansion = ned.imuExpansion.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return "Expansion: " + String.valueOf(angleExpansion.firstAngle) + "\nAngle: Control: " + String.valueOf(angleControl.firstAngle);
    }

    public double readDoubleAngle() {
        angleControl = ned.imuControl.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angleControl.firstAngle;
    }
}
