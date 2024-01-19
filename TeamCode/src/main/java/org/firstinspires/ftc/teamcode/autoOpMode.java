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
    //CLOSE LIBERALS
    //-------------
    private VisionPortal visionPortal;
	private ColourMassDetectionProcessor colourMassDetectionProcessor;

    // the current range set by lower and upper is the full range
	// HSV takes the form: (HUE, SATURATION, VALUE)
	// which means to select our colour, only need to change HUE
	// the domains are: ([0, 180], [0, 255], [0, 255])
	// this is tuned to detect red, so you will need to experiment to fine tune it for your robot
	// and experiment to fine tune it for blue
	Scalar lower = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
	Scalar upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
	double minArea = 100; // the minimum area for the detection to consider for your prop
		
	colourMassDetectionProcessor = new ColourMassDetectionProcessor(
				lower,
				upper,
				() -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
				() -> 213, // the left dividing line, in this case the left third of the frame
				() -> 426 // the left dividing line, in this case the right third of the frame
		);
		visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
				.addProcessor(colourMassDetectionProcessor)
				.build();
		
		// you may also want to take a look at some of the examples for instructions on
		// how to have a switchable camera (switch back and forth between two cameras)
		// or how to manually edit the exposure and gain, to account for different lighting conditions
		// these may be extra features for you to work on to ensure that your robot performs
		// consistently, even in different environments

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor mainArmDrive = null; // main
    private DcMotor slideDrive = null;
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


        mainArmDrive = hardwareMap.get(DcMotor.class, "main_arm_drive");
        slideDrive = hardwareMap.get(DcMotor.class, "slide_drive");
        /*
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

        move(1, 1000);
        turn(1, 1);
        move(1, 3);

    }

    public void move(double pow, long dur){
        leftFrontDrive.setPower(pow);
        leftBackDrive.setPower(pow);
        rightFrontDrive.setPower(pow);
        rightBackDrive.setPower(pow);
        sleep(dur);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void turn(double dir, long dur){
        leftFrontDrive.setPower(dir);
        leftBackDrive.setPower(dir);
        rightFrontDrive.setPower(-dir);
        rightBackDrive.setPower(-dir);
        sleep(dur);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void setArm(double pow){
        mainArmDrive.setPower(pow);
    }
    public void setSlide(double pow){
        slideDrive.setPower(pow);
    }
}
