package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "RamRovAutoOp")
public class autoOpMode extends LinearOpMode{

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    private DcMotor mainArmDrive = null; // main
    private DcMotor secondaryArmDrive = null;
    private Servo leftClawDrive = null;
    private Servo rightClawDrive = null;
    private Servo droneDrive = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        /*
        mainArmDrive = hardwareMap.get(DcMotor.class, "main_arm_drive");
        secondaryArmDrive = hardwareMap.get(DcMotor.class, "secondary_arm_drive");
        leftClawDrive = hardwareMap.get(Servo.class, "left_claw_drive");
        rightClawDrive = hardwareMap.get(Servo.class, "right_claw_drive");
        */
        droneDrive = hardwareMap.get(Servo.class, "drone_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
/*
        mainArmDrive.setDirection(DcMotor.Direction.FORWARD);
        secondaryArmDrive.setDirection(DcMotor.Direction.FORWARD);
        leftClawDrive.setDirection(Servo.Direction.FORWARD);
        rightClawDrive.setDirection(Servo.Direction.FORWARD);
*/
        droneDrive.setDirection(Servo.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(1);
            leftBackDrive.setPower(1);
            rightBackDrive.setPower(1);


            sleep(500);

            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
    }
}
