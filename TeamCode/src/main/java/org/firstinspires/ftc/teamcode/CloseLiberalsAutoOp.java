package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ColourMassDetectionOpMode;


@Autonomous(name = "CloseLiberalsAutoOp")
public class autoOpMode extends LinearOpMode{
    //CLOSE LIBERALS
    //-------------
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private Servo mainArmDrive = null; // main
    private DcMotor slideDrive = null;
    private Servo leftClawDrive = null;
    private Servo rightClawDrive = null;
    private Servo droneDrive = null;
    @Override
    public void runOpMode() {

	slideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	// the current range set by lower and upper is the full range
	// HSV takes the form: (HUE, SATURATION, VALUE)
	// which means to select our colour, only need to change HUE
	// the domains are: ([0, 180], [0, 255], [0, 255])
	// this is tuned to detect red, so you will need to experiment to fine tune it for your robot
	// and experiment to fine tune it for blue
	Scalar lower = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
	Scalar upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
	double minArea = 100; // the minimum area for the detection to consider for your prop
		
	// colourMassDetectionProcessor = new ColourMassDetectionProcessor(
	// 			lower,
	// 			upper,
	// 			() -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
	// 			() -> 213, // the left dividing line, in this case the left third of the frame
	// 			() -> 426 // the left dividing line, in this case the right third of the frame
	// 	);
	// 	visionPortal = new VisionPortal.Builder()
	// 			.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
	// 			.addProcessor(colourMassDetectionProcessor)
	// 			.build();
		
		// you may also want to take a look at some of the examples for instructions on
		// how to have a switchable camera (switch back and forth between two cameras)
		// or how to manually edit the exposure and gain, to account for different lighting conditions
		// these may be extra features for you to work on to ensure that your robot performs
		// consistently, even in different environments
	    
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");


        mainArmDrive = hardwareMap.get(Servo.class, "main_arm_drive");
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
        mainArmDrive.setDirection(Servo.Direction.FORWARD);
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
	setArm(0);
	setClaw(1); // closes the claw on preloaded pixels hopefully
	move(1, 1000); //moves up to the lines

	//hardcoding until we get cam to work
	turn(1, 1000);
	move(1, 3000);
	setArm(1);
	setSlide(1);
	sleep(1000);
	setSlide(0);
	setClaw(0);
	    
	// gets the recorded prop position
		ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();
		
		// now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
		// if it is UNFOUND, you can manually set it to any of the other positions to guess
		if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
			recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
		}
		
		// now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
	    	// turn toward the line with the prop
		// switch (recordedPropPosition) {
		// 	case LEFT:
		// 		turn(1, 1000)
		// 		break;
		// 	case UNFOUND: // we can also just add the unfound case here to do fallthrough intstead of the overriding method above, whatever you prefer!
		// 	case MIDDLE:
		// 		break;
		// 	case RIGHT:
		// 		// code to do if we saw the prop on the right
		// 		turn(-1, 1000)
		// }
	 //    	//place pixel
	 //    	setClaw(0);
	
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

    public void setArm(double pos){
        mainArmDrive.setPosition(pos);
    }
    public void setSlide(double pow){
        slideDrive.setPower(pow);
    }

    public void setClaw(double pos){
	leftClawDrive.setPosition(pos);
	rightClawDrive.setPosition(pos);
    }
}
