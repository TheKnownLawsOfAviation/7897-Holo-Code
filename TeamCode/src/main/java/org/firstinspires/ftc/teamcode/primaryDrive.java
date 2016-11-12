package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * *Copyright (c) 2016 Robert Atkinson
 *
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without modification,
 *are permitted (subject to the limitations in the disclaimer below) provided that
 *the following conditions are met:
 *
 *Redistributions of source code must retain the above copyright notice, this list
 *of conditions and the following disclaimer.
 *
 *Redistributions in binary form must reproduce the above copyright notice, this
 *list of conditions and the following disclaimer in the documentation and/or
 *other materials provided with the distribution.
 *
 *Neither the name of Robert Atkinson nor the names of his contributors may be used to
 *endorse or promote products derived from this software without specific prior
 *written permission.
 *
 *
 *
 *
 *NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 *LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Holonomic9")
public class primaryDrive extends LinearOpMode {
        HardwarePushbot robot = new HardwarePushbot();
        DcMotor motor1;
        DcMotor motor2;
        DcMotor motor3;
        DcMotor motor4;


    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.dcMotor.get("motor_1");
        motor2 = hardwareMap.dcMotor.get("motor_2");
        motor3 = hardwareMap.dcMotor.get("motor_3");
        motor4 = hardwareMap.dcMotor.get("motor_4");
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.FORWARD);
        robot.init(hardwareMap);
        telemetry.addData("Say", "Holonomic Ready...");    //
        telemetry.update();
        waitForStart();
        String frontDirection = "Forward";
        double frontLeftPower = 0, frontRightPower = 0, backRightPower = 0, backLeftPower = 0;
        double motor1power, motor2power, motor3power, motor4power;
        while (opModeIsActive()) {
            //Holonomic Calculations
            double JLX = gamepad1.left_stick_x * 200;
            double JLY = gamepad1.left_stick_y * 200;
            double JRX = ((gamepad1.left_trigger - 0.5) * 2) * 50;
            if (JRX > -2 && JRX < 2) {
                JRX = 0;
            }
            //Holonomic Clac. End
            // Virtual Fronts Start
            if (gamepad1.dpad_up && !frontDirection.equals("Forward")){
                frontDirection = "Forward";

            }
            else if (gamepad1.dpad_left && !frontDirection.equals("Left")){
                frontDirection = "Left";
            }
            else if (gamepad1.dpad_down && !frontDirection.equals("Backward")) {
                frontDirection = "Backward";
            }
            else if (gamepad1.dpad_right && !frontDirection.equals("Right")){
                frontDirection = "Right";
            }
            motor3power = ((-JLY - JLX) + JRX) * 1;
            motor1power = ((JLY - JLX) + JRX) * 1;
            motor2power = ((JLY + JLX) + JRX) * 1;
            motor4power = ((-JLY + JLX) + JRX) * 1;
            if (frontDirection.equals("Forward")) {

                frontLeftPower = motor1power;
                frontRightPower = motor2power;
                backRightPower = motor3power;
                backLeftPower = motor4power;

            }
            else if (frontDirection.equals("Backward")) {

                frontLeftPower = motor3power;
                frontRightPower = motor4power;
                backRightPower = motor1power;
                backLeftPower = motor2power;

            }
            else if (frontDirection.equals("Left")) {

                frontLeftPower = motor4power;
                frontRightPower = motor1power;
                backRightPower = motor2power;
                backLeftPower = motor3power;

            }
            else if (frontDirection.equals("Right")) {

                frontLeftPower = motor2power;
                frontRightPower = motor3power;
                backRightPower = motor4power;
                backLeftPower = motor1power;

            }
            //Virtual Fronts End
            //Setting Power
            motor1.setPower(frontLeftPower);
            motor2.setPower(frontRightPower);
            motor3.setPower(backRightPower);
            motor4.setPower(backLeftPower);
            //Setting Power End
            //Telemetry
            telemetry.addData("Say", "JRX = " + JRX);
            telemetry.addData("Say", "Front Direction is " + frontDirection);
            telemetry.addData("Say", "Motor Powers = " + motor1power + "," + motor2power + ","  + motor3power + "," + motor4power);
            telemetry.update();
            //Telemetry End
            //
            robot.waitForTick(40);
        }
    }
}
