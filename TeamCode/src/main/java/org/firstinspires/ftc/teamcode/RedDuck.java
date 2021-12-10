package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="RedDuck", group="LinearOpMode")

public class RedDuck extends LinearOpMode {

    //Defined Constants
    public static long DRIVE_MOTOR_MAX_TICKS = 1100;

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();


    // Initialize all motors to null
    private DistanceSensor Distance = null;
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
        Distance = hardwareMap.get(DistanceSensor.class, "distance");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        Turret.setDirection(DcMotor.Direction.FORWARD);
        Duck.setDirection(DcMotor.Direction.FORWARD);
        Grabber.setDirection(CRServo.Direction.FORWARD);

        //Set ArmMotor Encoders
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Distance Sensor Initial Telemetry
        Distance = hardwareMap.get(DistanceSensor.class, "distance");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double AutoStarts = runtime.seconds();


//        while (opModeIsActive() && runtime.seconds() > 1.0) {
//            telemetry.addData("TURRET ENCODER", Turret.getCurrentPosition());
//            telemetry.addData("ARM ENCODER", ArmMotor.getCurrentPosition());
//            telemetry.addData("FrontLeftDrive", FrontLeftDrive.getCurrentPosition());
//            telemetry.addData("FrontRightDrive", FrontRightDrive.getCurrentPosition());
//            telemetry.addData("BackRightDrive", BackRightDrive.getCurrentPosition());
//            telemetry.addData("BackLeftDrive", BackLeftDrive.getCurrentPosition());
//            telemetry.update();
//        } //END OF TELEMETRY WHILE LOOP

        while (opModeIsActive() && runtime.seconds() > 1.0) {

            // Lift ArmMotor
            runArm(1, 400);

            //Strafe towards BC1(Bar Code 1/Mid)
            while (BackLeftDrive.getCurrentPosition() < DRIVE_MOTOR_MAX_TICKS * 5) {
                setStrafe(.5);
            }

            sleep(30000);
            //Start Sensor Loop

        } //END OF MAIN WHILE LOOP


    } //END OF OPMODE MAIN CODE
    //ArmMotor
    private void runArm(double power,int position) {
        ArmMotor.setTargetPosition(position);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setPower(power);
        while(ArmMotor.getCurrentPosition() < position-20
                || ArmMotor.getCurrentPosition() > position+20) {
            telemetry.addData("ARM ENCODER", ArmMotor.getCurrentPosition());
            telemetry.update();
            sleep(100);
        }
    }
    //Strafe
    private void setStrafe(double power) {
        FrontLeftDrive.setPower(-power);
        FrontRightDrive.setPower(-power);
        BackLeftDrive.setPower(power);
        BackRightDrive.setPower(power);
    }

} //END OF CLASS

