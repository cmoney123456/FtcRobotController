package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Billy" , group="Linear OpMode")

public class coreyPracticeTeleOp4 extends LinearOpMode {

    double power = 1;
    double intakePower;
    double wristPower = 0.75;
    double clawPosition = 1;
    double pivotPower = 0;//gamepad2.right_stick_y * 0.05;



    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    MechanumClass drive = new MechanumClass();
    IMUClass imu = new IMUClass();

    private static final int Max_Position = -2875;
    private static final int Max_Slide = -900;

    private boolean constantPower =false;

    @Override
    public void runOpMode() {

        drive.init(hardwareMap, false);
        imu.initIMU(hardwareMap);




        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double horizontal = gamepad1.left_stick_x;
            double vertical = -gamepad1.left_stick_y;
            double pivot = -gamepad1.right_stick_x;
            double slidePower = gamepad2.left_stick_y ;
            double linearPower = gamepad2.right_stick_x * 0.5;




            int currentPosition = drive.posistion();
            double curSlide = drive.slidePos();
            double linearPos = drive.linearPos();


            /*if (gamepad1.left_bumper) {
                intakePower = -1.0;
            } else if (gamepad1.right_bumper) {
                intakePower = 0.5;
            } else if (gamepad1.y) {
                intakePower = 0.0;
            }
            if (gamepad2.left_trigger > 0) {
                wristPower = 0.6;
            } else if (gamepad2.right_trigger > 0){
                wristPower = 0.1667;
            }*/

            if (gamepad2.left_trigger > 0.1){
                constantPower = !constantPower;
                while (gamepad2.left_trigger > 0.1 && opModeIsActive()){
                    sleep(500);
                }
            }
            if (constantPower) {

                    pivotPower = 0.3;
                    sleep(15000);

            }
            else {
                if (gamepad2.dpad_down){
                    pivotPower = gamepad2.right_stick_y * 0.3;
                }
                else if (gamepad2.dpad_up){
                    pivotPower = gamepad2.right_stick_y * 0.5;
                }
                else {
                    pivotPower = gamepad2.right_stick_y * 0.05;
                }
            }

            if (gamepad1.dpad_down){
                power = .5;
            }
            else {
                power = 1;
            }



            if (gamepad2.b){
                clawPosition = 0.5;
            }
            else if (gamepad2.y) {
                clawPosition = 0;

            }
            if (gamepad2.left_bumper) {
                wristPower = 0;
            }
            else if (gamepad2.right_bumper) {
                wristPower = 0.75;
            }
            if (gamepad2.a){
                intakePower = 1;
            }
            else if (gamepad2.x){
                intakePower = 0;
            }
            else if (gamepad2.dpad_right) {
                wristPower = 0.375;
            }

            if (currentPosition > Max_Position){
                if (gamepad2.right_stick_y != 0){
                    if (gamepad2.dpad_down) {
                        pivotPower = gamepad2.right_stick_y * 0.3;
                    }
                    else if (gamepad2.dpad_up){
                        pivotPower = gamepad2.right_stick_y * 0.5;
                    }
                    else if (gamepad2.dpad_left){
                        pivotPower = 0.3;
                    }
                }
                else {
                    pivotPower = 0;
                }

            }

            else {
                pivotPower = 0;
                telemetry.addLine("Max position reached!");
                telemetry.update();
            }
            if (currentPosition < 0 && currentPosition > -900){
                if (curSlide < -900){
                    slidePower = 0;
                }
            }

            else if (currentPosition < -2350){
                wristPower = 0.1667;
                if (curSlide < 0){
                    slidePower = 0.25;
                }
                else if (curSlide > -100 && gamepad2.left_stick_y > 0){
                    slidePower = 0;
                }

            }




            drive.teleOP( power,  pivot,  vertical,  horizontal,  pivotPower,  slidePower,  intakePower,  wristPower, currentPosition, linearPower, clawPosition, linearPos);


            telemetry.addData("linear",linearPower);
            telemetry.addData("linear Pos",linearPos);
            telemetry.addData("Drive Power",power);
            telemetry.addData("Lift Power",pivotPower);
            telemetry.addData("Slide Power",slidePower);
            telemetry.addData("intakePower",intakePower);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Target Limit",Max_Position);
            telemetry.addData("Slide Pos",curSlide);
            telemetry.addData("Slide Limit",Max_Slide);
            telemetry.addData("constant power",constantPower);
            telemetry.update();


        }
    }}