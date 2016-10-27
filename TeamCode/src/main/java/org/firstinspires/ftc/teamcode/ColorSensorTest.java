/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.steelhead.ftc.ColorPIDController;
import org.steelhead.ftc.HardwareSteelheadMainBot;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Sensor: Color Sensor", group = "Sensor")
public class ColorSensorTest extends LinearOpMode {

    private final double MAX_MOTOR_POWER = 0.25;
    private final double MIN_MOTOR_POWER = -0.25;
    private final double DRIVE_SPEED = 0.1;
    HardwareSteelheadMainBot robot = new HardwareSteelheadMainBot();
    ColorSensor colorSensor;
    ColorPIDController pidController;

    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_POWER), MAX_MOTOR_POWER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        double output = 0;

        robot.init(hardwareMap);

        robot.robotBackward();

        colorSensor = hardwareMap.colorSensor.get("color");
        pidController = new ColorPIDController(colorSensor, 3, 43);

        colorSensor.enableLed(false);
        colorSensor.enableLed(true);
        pidController.setPID(0.02, 0.0005, 0.1);

        waitForStart();

        pidController.enable();
        while (opModeIsActive()) {
            output = pidController.getOutput();
            robot.leftMotor_1.setPower(limit(DRIVE_SPEED + output));
            robot.leftMotor_2.setPower(limit(DRIVE_SPEED + output));
            robot.rightMotor_1.setPower(limit(DRIVE_SPEED - output));
            robot.rightMotor_2.setPower(limit(DRIVE_SPEED - output));

            telemetry.addData("Output: ", pidController.getOutput());
            telemetry.update();
        }
        colorSensor.enableLed(false);
        pidController.disable();

    }
}



