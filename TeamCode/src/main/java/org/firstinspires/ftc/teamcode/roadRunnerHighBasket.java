package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class roadRunnerHighBasket extends LinearOpMode {

    DcMotor linear;
    Servo claw;
    DcMotor slide;
    DcMotor lift;
    Servo wrist;
    Servo intake;



    // PID parameters
    double integral = 0;
    double lastError = 0;
    double Kp = 0.001; // Proportional gain
    double Ki = 0.00001; // Integral gain
    double Kd = 0.00001; // Derivative gain
    double targetPosition = 0; // Target position
    double slidetarPos = 0;
    double slideLastError = 0;
    double slideKp = 0.06;
    double slideKi = 0.0006;
    double slideKd = 0.0006;
    double slideIntegral = 0;
    double wristPos = 0.75;
    double intakePos =0;

    final double POSITION_TOLERANCE = 10; // Acceptable error (tolerance) for reaching the target

    ElapsedTime PIDTimer = new ElapsedTime();




    public void init(HardwareMap hwMap, boolean autoMode){
        linear = hwMap.get(DcMotor.class,"linearMotor");
        claw = hwMap.get(Servo.class,"claw");
        slide = hwMap.get(DcMotor.class,"armMotor");
        lift = hwMap.get(DcMotor.class,"liftMotor");
        wrist = hwMap.get(Servo.class,"wrist");
        intake = hwMap.get(Servo.class,"intake");

    }

    //double wristPower = 0.75;
    //double clawPosition = 1;


    public void runOpMode() {

        init(hardwareMap,false);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // Ensure motor stops moving when power is set to 0
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the encoder to zero
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // Ensure motor stops moving when power is set to 0
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the encoder to zero
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPos = new Pose2d(-36,-64,Math.toRadians(90));

        drive.setPoseEstimate(startPos);


        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                        .splineTo(new Vector2d(-57,-52),Math.toRadians(225))
                        .build();


        highBasket(0,0);
        waitForStart();

        if(isStopRequested()) return;

        //drive.followTrajectory(traj1);
        highBasket(-2100,0);




    }

    private void highBasket(double targetPosition, double slidetarPos) {

        double motorPower = pidControl(targetPosition);
        double slidePower = slidepidControl(slidetarPos);


        motorPower = Math.max(-1, Math.min(1, motorPower));
        slidePower = Math.max(-1, Math.min(1,slidePower));

        // Set the motor power
        lift.setPower(motorPower * 0.15);
        slide.setPower(slidePower * 0.1);
        wrist.setPosition(wristPos);
        intake.setPosition(intakePos);
        while ((opModeIsActive() && slide.isBusy()) || (opModeIsActive() && slide.isBusy())){
            telemetry.addData("lift pos", lift.getCurrentPosition());
            telemetry.addData("slide pos", slide.getCurrentPosition());
            telemetry.update();
        }

    }

    private void openClaw(double pos, int tarPos) {
        claw.setPosition(pos);
        linear.setTargetPosition(tarPos);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(0.5);


        while (opModeIsActive() && linear.isBusy()){
            telemetry.addData("ClawPos",pos);
            telemetry.update();
        }


    }


    private void moveArmUp(int tarPos) {
        linear.setTargetPosition(tarPos);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(-0.5);
        claw.setPosition(0);


        while (opModeIsActive() && linear.isBusy()){
            telemetry.addData("Slide Position",linear.getCurrentPosition());
            telemetry.update();
        }

        linear.setPower(0);

    }
    private void moveArmDown(int tarPos){
        linear.setTargetPosition(tarPos);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(0.5);
        claw.setPosition(0);

        while(opModeIsActive() && linear.isBusy()){
            telemetry.addData("Slide Position",linear.getCurrentPosition());
            telemetry.update();
        }
        claw.setPosition(0.5);
    }

    private double pidControl(double targetPosition) {
        double currentPosition = lift.getCurrentPosition();
        double error = targetPosition - currentPosition;
        double deltaTime = PIDTimer.time(); // Time since last call

        if (deltaTime > 0) {
            // Calculate integral and derivative
            integral += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;

            // PID output
            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

            // Update last error and reset timer for next calculation
            lastError = error;
            PIDTimer.reset();


            return output;
        } else {
            // If deltaTime is 0, return 0 to prevent division by zero (for the first loop iteration)
            return 0;
        }

    }
    // PID control function
    private double slidepidControl(double slidetarPos) {
        double slideCurPos = slide.getCurrentPosition();
        double deltaTime = PIDTimer.time(); // Time since last call
        double slideError = slidetarPos - slideCurPos;

        if (deltaTime > 0) {
            // Calculate integral and derivative
            slideIntegral += slideError * deltaTime;
            double slideDerivative = (slideError - slideLastError) / deltaTime;

            // PID output
            double slideOutput = (slideKp * slideError) + (slideKi * slideIntegral) + (slideKd * slideDerivative);

            // Update last error and reset timer for next calculation
            slideLastError = slideError;
            PIDTimer.reset();


            return slideOutput;
        } else {
            // If deltaTime is 0, return 0 to prevent division by zero (for the first loop iteration)
            return 0;
        }
    }
    // This method can be added to check if the motor is near the target and stop PID control when the target is reached
    private boolean isAtTargetPosition(double targetPosition) {
        double currentPosition = lift.getCurrentPosition();
        return Math.abs(targetPosition - currentPosition) <= POSITION_TOLERANCE;
    }



}