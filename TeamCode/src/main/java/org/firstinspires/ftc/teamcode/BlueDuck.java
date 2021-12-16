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

@Autonomous(name="BlueDuck", group="LinearOpMode")

public class BlueDuck extends LinearOpMode {

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
        Duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            runArm(1, 800);
            sleep(500);


            //Strafe towards BC1(Bar Code 1/Mid)
            while (BackLeftDrive.getCurrentPosition() > -DRIVE_MOTOR_MAX_TICKS * 1.75) {
                setStrafe(-.5);
            }
            setStrafe(0);
            //Start Sensor Loop
            Boolean[] array = new Boolean[3];
            array[0] = false;
            array[1] = false;
            array[2] = false;
            //Sense BCE1
            if(Distance.getDistance(DistanceUnit.CM) < 20) {
                array[1] = true;
            }
            telemetry.addData("Middle Location", array[1]);
            telemetry.update();

            //Froward to BCE2
            resetDriveEncoders();
            while (BackLeftDrive.getCurrentPosition() < DRIVE_MOTOR_MAX_TICKS * 0.675) {
                setDrive(.2);
            }
            setDrive(0);
            //Sense BCE2
            if(Distance.getDistance(DistanceUnit.CM) < 20) {
                array[0] = true;
            }
            telemetry.addData("Last Location", array[0]);
            telemetry.update();
            //State BCE3
            if(array[1] == false && array[0] == false) {
                array[2] = true;
            }

            // Rotate Test
            resetDriveEncoders();
            while (BackLeftDrive.getCurrentPosition() < DRIVE_MOTOR_MAX_TICKS * 1.45) {
                setRotate(-.5);
            }
            setRotate(0);

            //Forward to wall
            resetDriveEncoders();
            while (BackLeftDrive.getCurrentPosition() < DRIVE_MOTOR_MAX_TICKS * 2.25) {
                setStrafe(.5);
            }
            setStrafe(0);

            //Forward to Duck
            resetDriveEncoders();
            while (BackLeftDrive.getCurrentPosition() > -DRIVE_MOTOR_MAX_TICKS * 0.5) {
                setDrive(-.5);
            }
            setDrive(0);

            //Deliver Duck
            Duck.setPower(0.3);
            sleep(3700);
            Duck.setPower(0);

            //Strafe into wall
            resetDriveEncoders();
            while (BackLeftDrive.getCurrentPosition() < DRIVE_MOTOR_MAX_TICKS * 0.15) {
                setStrafe(.5);
            }
            setStrafe(0);

            //Move Back to Cargo Loader
            resetDriveEncoders();
            while (BackLeftDrive.getCurrentPosition() > -DRIVE_MOTOR_MAX_TICKS * 4.8) {
                setStrafe(-.5);
            }
            setStrafe(0);

            //Rotate to Drop off Cargo
            resetDriveEncoders();
            while (BackLeftDrive.getCurrentPosition() > -DRIVE_MOTOR_MAX_TICKS * 1.45) {
                setRotate(.5);
            }
            setRotate(0);

            if (array[1] == true) {
                runArm(1, 1100);
                resetDriveEncoders();
                while (BackLeftDrive.getCurrentPosition() > -DRIVE_MOTOR_MAX_TICKS * 1.05) {
                    setStrafe(-.5);
                }
                setStrafe(0);
            } else if (array[0] == true) {
                runArm(1, 625);
                resetDriveEncoders();
                while (BackLeftDrive.getCurrentPosition() > -DRIVE_MOTOR_MAX_TICKS * 1.15) {
                    setStrafe(-.5);
                }
                setStrafe(0);
            } else if (array[2] == true) {
                runArm(1, 1600);
                resetDriveEncoders();
                while (BackLeftDrive.getCurrentPosition() > -DRIVE_MOTOR_MAX_TICKS * 1) {
                    setStrafe(-.5);
                }
                setStrafe(0);
            }


            //Deliver Cargo
            Grabber.setPower(-.5);
            sleep(2000);
            Grabber.setPower(0);

            //Strafe to Wall
            resetDriveEncoders();
            while (BackLeftDrive.getCurrentPosition() < DRIVE_MOTOR_MAX_TICKS * 2.35) {
                setStrafe(.5);
            }
            setStrafe(0);

            //Move Back to Station
            resetDriveEncoders();
            while (BackLeftDrive.getCurrentPosition() > -DRIVE_MOTOR_MAX_TICKS * 4) {
                setDrive(-.5);
            }
            setDrive(0);
            sleep(30000);
        }
        sleep(30000);
    } //END OF MAIN WHILE LOOP


    //END OF OPMODE MAIN CODE
    //ArmMotor
    private void runArm(double power,int position) {
        ArmMotor.setTargetPosition(position);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setPower(power);
        while(ArmMotor.getCurrentPosition() < position-10
                || ArmMotor.getCurrentPosition() > position+10) {
            telemetry.addData("ARM ENCODER", ArmMotor.getCurrentPosition());
            telemetry.update();
            sleep(100);
        }
    }
    //Strafe
    private void setStrafe(double power) {
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeftDrive.setPower(-power);
        FrontRightDrive.setPower(-power);
        BackLeftDrive.setPower(power);
        BackRightDrive.setPower(power);
    }
    // Drive
    private void setDrive(double power) {
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeftDrive.setPower(power);
        FrontRightDrive.setPower(-power);
        BackLeftDrive.setPower(power);
        BackRightDrive.setPower(-power);
    }

    //Rotate Right
    private void setRotate(double power) {
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeftDrive.setPower(-power);
        FrontRightDrive.setPower(-power);
        BackLeftDrive.setPower(-power);
        BackRightDrive.setPower(-power);
    }

    //Reset The Encoders
    private void resetDriveEncoders() {
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
} //END OF CLASS

