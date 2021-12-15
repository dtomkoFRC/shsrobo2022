package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="ResetArm", group="Linear Opmode")


public class ResetArm extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();

    // Initialize all motors to null
    public DcMotor ArmMotor = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        ArmMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Set arm power
            ArmMotor.setPower(-gamepad2.left_stick_y*0.5);
        }
    }
}
