package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="NewMainJoy", group="Linear Opmode")

public class NewMainJoy extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Initialize all motors to null
    public DcMotor FrontLeftDrive = null;
    public DcMotor FrontRightDrive = null;
    public DcMotor BackLeftDrive = null;
    public DcMotor BackRightDrive = null;
    public DcMotor ArmMotor = null;
    public DcMotor Turret = null;
    public DcMotor Duck = null;
    public CRServo Grabber = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        Turret = hardwareMap.get(DcMotor.class, "Turret");
        Duck = hardwareMap.get(DcMotor.class, "Duck");
        Grabber = hardwareMap.get(CRServo.class, "Grabber");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        Turret.setDirection(DcMotor.Direction.FORWARD);
        Duck.setDirection(DcMotor.Direction.FORWARD);
        Grabber.setDirection(CRServo.Direction.FORWARD);
      
        //Set ArmMotor Encoders
        ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Init Drivetrain Variables
        double x;
        double y;
        double r;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drivetrain Setup
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            r = gamepad1.right_stick_x;
            //Slow Drivetrain Mode
            if(gamepad1.left_bumper) {
                x *= 0.25;
                y *= 0.25;
                r *= 0.25;
            }
            //Drive Command
            FrontLeftDrive.setPower(y - x - r);
            FrontRightDrive.setPower(y + x + r);
            BackLeftDrive.setPower(y + x - r);
            BackRightDrive.setPower(y - x + r);

            //True Straight
            //Forward
            while(gamepad1.dpad_up) {
                FrontLeftDrive.setPower(0.5);
                FrontRightDrive.setPower(0.5);
                BackLeftDrive.setPower(0.5);
                BackRightDrive.setPower(0.5);
                sleep(10);
            }
            //Backward
            while(gamepad1.dpad_down) {
                FrontLeftDrive.setPower(-0.5);
                FrontRightDrive.setPower(-0.5);
                BackLeftDrive.setPower(-0.5);
                BackRightDrive.setPower(-0.5);
                sleep(10);
            }
            
            //Arm Motor Movement
            ArmMotor.setPower(-gamepad2.left_stick_y * 0.5);

            //Turret Motor Movement
            Turret.setPower(gamepad2.right_stick_x * 0.25);

            //Duck Motor
            if(gamepad2.right_trigger > 0) {
                Duck.setPower(-0.6);
            }
            Duck.setPower(0);

            //Grabber Servo
            Grabber.setPower(gamepad2.right_stick_y);

            //PRINT OUTS
            telemetry.addLine("She's ready to rip");
            telemetry.update();
        }
    }
}