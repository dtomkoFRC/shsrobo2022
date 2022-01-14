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

@TeleOp(name="NewMain", group="Linear Opmode")


public class NewMain extends LinearOpMode {
  
// Constants Arm
static final int ARMUP = 400;
static final int ARMDOWN = 400;
static final double ARMPOW = 0.4;
// Constants Turret
static final int TURRETRIGHT = 50;
static final int TURRETLEFT = 50;
static final double TURRETPOW = 0.75;
//Dapd Commands

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
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        ArmMotor.setDirection(DcMotor.Direction.FORWARD);
        Turret.setDirection(DcMotor.Direction.FORWARD);
        Duck.setDirection(DcMotor.Direction.FORWARD);
        Grabber.setDirection(CRServo.Direction.FORWARD);
      
        //Set ArmMotor Encoders
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Set inital values for ticks outside of loop
        int armticks = 0;
        int turretticks = 0;
        double armpower = 0.3;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Drivetrain control
            // Strafe
            double x = gamepad1.left_stick_x;
//            // Forward
            double y = -gamepad1.left_stick_y;
            // Rotate
            double r = gamepad1.right_stick_x;
            // Forward

            boolean dpadupPreviousState = false;
            boolean straightForward = false;

//            //Define faster mode
//            if(gamepad2.dpad_up && !dpadupPreviousState) {
//                straightForward = !straightForward;
//            }
//            if(!straightForward) {
//                FrontRightDrive.setPower(0.8);
//                FrontRightDrive.setPower(-0.8);
//                BackLeftDrive.setPower(0.8);
//                BackRightDrive.setPower(-0.8);
//            }
//            else {
//                y = -gamepad1.left_stick_y;
//            }


            double leftPower = x;
            double rightPower = y - r;
            // double sidePower = y + r;
            
            // Arm motor movement
            if (gamepad2.dpad_up) {
                armticks += ARMUP;
                armpower = ARMPOW;
                sleep(500);
            } else if (gamepad2.dpad_down) {
                armticks -= ARMDOWN;
                armpower = ARMPOW;
                sleep(500);
            } else  if (gamepad2.a) {
                armticks = 0;
                armpower = 0.6;
                sleep(500);
            }

            //telemetry.addData("armticks", armticks);
            //    telemetry.update();
            //telemetry.addLine("\n").addData("Actual Encoder Pos", ArmMotor.getCurrentPosition());
            //telemetry.update();
            // Arm run command
            runArm(armpower, armticks);


//            // Turret movement
//            if (gamepad2.dpad_right)
//            {
//                  telemetry.addData("Stats", "dpadright");
//                  telemetry.update();
//                turretticks += TURRETRIGHT;
//                sleep(500);
//            } else if (gamepad2.dpad_left)
//            {
//                turretticks -= TURRETLEFT;
//                telemetry.addData("Stats", "dpad_left");
//                telemetry.update();
//                sleep(500);
//            }
//            telemetry.addData("turretticks", turretticks);
//                telemetry.update();
//            Turret.setTargetPosition(turretticks);
//            telemetry.addData("Actual T Encoder Pos", Turret.getCurrentPosition());
//            telemetry.update();
            // Turret run command
//            runTurret(TURRETPOW, turretticks);
            // Turret Parameters

            int TurretPosition = Turret.getCurrentPosition();
            //telemetry.addLine("\n").addData("TURRET ENCODER", TurretPosition);
            //telemetry.update();

            //Clockwise
            if (gamepad2.left_stick_x > 0 ){
                if(TurretPosition < 290) {
                    Turret.setPower(gamepad2.left_stick_x * .25);
                    sleep(10);
                }
            }
            //Counter Clockwise
            else if(gamepad2.left_stick_x < 0 ){
                if(TurretPosition > -150 ) {
                    Turret.setPower(gamepad2.left_stick_x * .25);
                    sleep(10);
                }
            }
            else if (gamepad2.b){
                turretticks = 150;
                runTurret(0.5,turretticks);
                sleep(10);
            }
            else {
                Turret.setPower(0);
            }

            // duck motor
            double d = gamepad2.right_trigger;

//            // grabber
            double g = gamepad2.right_stick_y;
//
            

            
            // Limit Wall
           FrontLeftDrive.setPower(y);
           FrontRightDrive.setPower(-y);
           BackLeftDrive.setPower(y);
           BackRightDrive.setPower(-y);
           // turning
           FrontLeftDrive.setPower(-r);
           FrontRightDrive.setPower(-r);
           BackLeftDrive.setPower(-r);
           BackRightDrive.setPower(-r);
           //Strafe
           FrontLeftDrive.setPower(-x);
           FrontRightDrive.setPower(-x);
           BackLeftDrive.setPower(x);
           BackRightDrive.setPower(x);
           // Duck
           Duck.setPower(-d);
          // Grabber
           Grabber.setPower(g);

           //PRINT OUTS
            telemetry.addData("TURRET ENCODER", TurretPosition);
            telemetry.addData("ARM ENCODER", ArmMotor.getCurrentPosition());
            telemetry.addData("TARGET ARM TICKS", armticks);
            telemetry.update();

         //Duck speed limit
            if(gamepad2.right_trigger > 0.00000001) {
                    Duck.setPower(-0.00000001);
                }
                
                if(gamepad2.right_trigger < 0.00000001) {
                    Duck.setPower(0);
                }


            
            
        }}
    private void setMotorsForward(double power) {
        FrontRightDrive.setPower(power);
        FrontLeftDrive.setPower(power);
    }    
   
   private void runArm(double power, int position) {
       ArmMotor.setTargetPosition(position);
       ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       ArmMotor.setPower(power);
       
   }
//    private void runTurret(double power, int position) {
//       Turret.setTargetPosition(position);
//       Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//       Turret.setPower(power);
//
//   }
    
    
    private void setMotorsRotate(double power) {
        FrontRightDrive.setPower(power);
        FrontLeftDrive.setPower(power);
        BackLeftDrive.setPower(power);
        BackRightDrive.setPower(power);
    // NonDriveTrain
        Duck.setPower(power);
        Grabber.setPower(power);
    
    }
    private void runTurret(double power, int position) {
        Turret.setTargetPosition(position);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Turret.setPower(power);
    }

    }
