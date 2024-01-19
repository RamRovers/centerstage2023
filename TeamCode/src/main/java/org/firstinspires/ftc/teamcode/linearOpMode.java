package org.firstinspires.ftc.teamcode;


/*
Copyright 2019 FIRST Tech Challenge Team Team 1

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RamRovTeleOp")
public class linearOpMode extends LinearOpMode {

    // Declare OpMode members for each of the motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null, leftBackDrive = null;
    private DcMotor rightFrontDrive = null, rightBackDrive = null;

    private DcMotor mainArmDrive = null;
    private DcMotor slideDrive = null;
    private Servo leftClawDrive = null;
    private Servo rightClawDrive = null;

    private Servo droneDrive = null;
    //private Servo armHolder = null;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");

        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        mainArmDrive = hardwareMap.get(DcMotor.class, "main_arm_drive");
        slideDrive = hardwareMap.get(Servo.class, "slide_drive");
        leftClawDrive = hardwareMap.get(Servo.class, "left_claw_drive");
        rightClawDrive = hardwareMap.get(Servo.class, "right_claw_drive");

        droneDrive = hardwareMap.get(Servo.class, "drone_drive");
        //armHolder = hardwareMap.get(Servo.class, "arm_holder");

        // set rotating directions for the motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        mainArmDrive.setDirection(DcMotor.Direction.FORWARD);
        slideDrive.setDirection(Servo.Direction.FORWARD);
        leftClawDrive.setDirection(Servo.Direction.FORWARD);
        rightClawDrive.setDirection(Servo.Direction.FORWARD);

        droneDrive.setDirection(Servo.Direction.REVERSE);
        //armHolder.setDirection(Servo.Direction.REVERSE);
        boolean reversed = false;
        double gear = 1;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double dronePosition = 1;
        double holderPosition = 1;

        double wristPosition = 0;
        double lClawPosition = 1;
        double rClawPosition = 1;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //reverse rotation moment
            if(gamepad1.y && reversed){
                reversed = false;
                leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
                rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            } else if(gamepad1.y && !reversed){
                reversed = true;
                leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
                rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            }

            droneDrive.setPosition(dronePosition);
            //armHolder.setPosition(holderPosition);

            double max;

            //putting this up here might be bad practice but idc
            if(gamepad1.right_bumper && gear < 1){
                gear += 0.2;
            } else if(gamepad1.left_bumper && gear > 0){
                gear -= 0.2;
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double leftFrontPower = -gamepad1.left_stick_y * gear;  // Note: pushing stick forward gives negative value
            double leftBackPower = -gamepad1.left_stick_y * gear;
            double rightFrontPower = gamepad1.right_stick_y * gear;
            double rightBackPower = gamepad1.right_stick_y * gear;

            double mArmPower = -gamepad2.left_stick_y;

            if (gamepad2.right_stick_y < 0 && wristPosition < 1){
                wristPosition += 0.1;
            } else if(gamepad2.right_stick_y > 0 && wristPosition > 0){
                wristPosition -= 0.1;
            }

            if (gamepad2.left_trigger > 0){
                lClawPosition = 1; // close
            } else if(gamepad2.left_bumper){
                lClawPosition = 0; // open
            } // if

            if (gamepad2.right_trigger > 0){
                rClawPosition = 1; // close
            } else if(gamepad2.right_bumper) {
                rClawPosition = 0; // open
            }

            if(gamepad2.x){
                lClawPosition = 0;
                rClawPosition = 0;
            } else if(gamepad2.a){
                lClawPosition = 1;
                rClawPosition = 1;
            }


            //launch drone
            if(gamepad1.b){
                dronePosition = 0;
            }

            //drop arm holder
            if(gamepad2.dpad_down){
                holderPosition = 0;
            }
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.

            /* not using
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            */

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

            mainArmDrive.setPower(mArmPower);
            wristDrive.setPosition(wristPosition);
            leftClawDrive.setPosition(lClawPosition);
            rightClawDrive.setPosition(rClawPosition);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Front", leftFrontPower);
            telemetry.addData("Left Back", leftBackPower);
            telemetry.addData("Right Front", rightFrontPower);
            telemetry.addData("Right Back", rightBackPower);
            telemetry.addData("plane!!!", gamepad1.b);
            telemetry.addData("holder dropped", gamepad2.dpad_down);
            telemetry.addData("current gear: ", gear);
            telemetry.update();
        }
    }
}
