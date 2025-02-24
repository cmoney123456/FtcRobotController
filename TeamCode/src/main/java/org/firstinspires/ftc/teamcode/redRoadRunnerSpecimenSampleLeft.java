package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Red Left Speci and Sample", group = "A")
public class redRoadRunnerSpecimenSampleLeft extends LinearOpMode {
    DcMotor testMotor;
    DcMotor slideMotor;
    Servo wrist;
    CRServo intakeLeft;
    CRServo intakeRight;

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
    double intakePos;

    final double POSITION_TOLERANCE = 10; // Acceptable error (tolerance) for reaching the target

    ElapsedTime PIDTimer = new ElapsedTime();


    DcMotor linear;
    Servo claw;

    public void init(HardwareMap hwMap, boolean autoMode){
        linear = hwMap.get(DcMotor.class,"linearMotor");
        claw = hwMap.get(Servo.class,"claw");
    }

    //double wristPower = 0.75;
    //double clawPosition = 1;


    public void runOpMode() {


        testMotor = hardwareMap.dcMotor.get("liftMotor");
        testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // Ensure motor stops moving when power is set to 0
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the encoder to zero
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Use encoder feedback for PID control
        slideMotor = hardwareMap.dcMotor.get("armMotor");
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // Ensure motor stops moving when power is set to 0
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the encoder to zero
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist = hardwareMap.servo.get("wrist");
        intakeLeft = hardwareMap.crservo.get("intakeLeft");
        intakeRight = hardwareMap.crservo.get("intakeRight");


        init(hardwareMap,false);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPos = new Pose2d(-24,-64,Math.toRadians(90));

        drive.setPoseEstimate(startPos);





        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(-8,-38),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(6.5,SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-50,-48),Math.toRadians(90),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                        .forward(12,SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                        .splineTo(new Vector2d(-51.5,-51.5),Math.toRadians(225),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();




        claw.setPosition(0);
        waitForStart();

        if(isStopRequested()) return;

        Thread moveSlideThread = new Thread(new Runnable() {
            public void run() {
                moveArmUp(-2350);
            }
        });



        moveSlideThread.start(); // Start the thread for moving the slide
        drive.followTrajectory(traj1);

        try {
            moveSlideThread.join(); // Wait for the slide movement to complete
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        moveArmUp(-2350);
        drive.followTrajectory(traj2);
        moveArmDown(-1200);
        openClaw(0.5,-1000);
        drive.followTrajectory(traj3);
        while(opModeIsActive()) {
            raiseArm(-550, 0, 0);
            if (testMotor.getCurrentPosition() < - 525){
                //raiseArm(-500,0,0);
                break;
            }
        }
        while (opModeIsActive()){
            raiseArm(0,0,0.85);
            extendSlide(-325);
            if (testMotor.getCurrentPosition() > -150){
                raiseArm(0,0,0.85);
                break;
            }
        }
        drive.followTrajectory(traj4);
        while (opModeIsActive()){
            raiseArm(0,0,0);
            if (testMotor.getCurrentPosition() > -100){
                raiseArm(0,0,0);
                break;
            }
        }
        drive.followTrajectory(traj5);
        while (opModeIsActive()){
            raiseArm(-2000,0,0);
            if (testMotor.getCurrentPosition() < -1950) {
                raiseArm(-2000,0.25,0);
                break;
            }
        }
        while (opModeIsActive()){
            extendSlide(-2100);
            if (slideMotor.getCurrentPosition() < -2050){
                extendSlide(-2100);
                raiseArm(-2000,0.25,-0.5);
                break;
            }

        }
        while (opModeIsActive()){
            extendSlide(-2100);
            raiseArm(-2000,0.25,-0.5);
            if (testMotor.getCurrentPosition() < -1900){
                break;
            }
        }
        while (opModeIsActive()){
            raiseArm(-2100,0.25,0);
            extendSlide(0);
            if (slideMotor.getCurrentPosition() > -15){
                break;
            }
        }
        while (opModeIsActive()){
            raiseArm(0,0,0);
            if (testMotor.getCurrentPosition() > 15){
                break;
            }
        }
        /*while (opModeIsActive()){
            raiseArm(-2100,0.25,0);
            if (wrist.getPosition() == 0.25){
                raiseArm(-2100,0.25,-0.5);
                break;
            }
        }*/


    }

    private void raiseArm(double armPos, double wristFlip, double intakeSpeed) {
        double motorPower = pidControl(targetPosition);
        double slidePower;
        double slidePos = slideMotor.getCurrentPosition();


        targetPosition = armPos;
        if (testMotor.getCurrentPosition() < armPos+100) {
            slidetarPos = slidePos;
            if (slideMotor.getCurrentPosition() < slidePos+100) {
                wristPos = wristFlip;
                sleep(500);
                intakePos = intakeSpeed;
            }
        }


        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        // Set the motor power
        if (testMotor.getCurrentPosition() < -1600) {
            testMotor.setPower(motorPower * 0.35);
        } else {
            testMotor.setPower(motorPower * 0.75);
        }
        if (slideMotor.getCurrentPosition() < -1500) {
           // slideMotor.setPower(slidePower * 0.1);
        } else {
            //slideMotor.setPower(slidePower * 0.1);
        }

        wrist.setPosition(wristPos);
        intakeLeft.setPower(-intakePos);
        intakeRight.setPower(intakePos);

        telemetry.addData("Motor Position", testMotor.getCurrentPosition());
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("Slide Motor Position", slideMotor.getCurrentPosition());
        telemetry.addData("Slide Target Position", slidetarPos);
//        telemetry.addData("Slide Motor curPos", slideMotor.getCurrentPosition());
        telemetry.addData("Kp", Kp);
        telemetry.addData("Ki", Ki);
        telemetry.addData("Kd", Kd);
        telemetry.addData("slide Kp", slideKp);
        telemetry.addData("slide Ki", slideKi);
        telemetry.addData("slide Kd", slideKd);
        telemetry.update();
    }
    private double pidControl(double targetPosition) {
        double currentPosition = testMotor.getCurrentPosition();
        double error = targetPosition - currentPosition;
        double deltaTime = PIDTimer.time();

        if (deltaTime > 0) {
            integral += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;
            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

            lastError = error;
            PIDTimer.reset();

            return output;
        } else {
            return 0;
        }
    }

    /*private double slidepidControl(double slidetarPos) {
        double slideCurPos = slideMotor.getCurrentPosition();
        double deltaTime = PIDTimer.time();
        double slideError = slidetarPos - slideCurPos;

        if (deltaTime > 0) {
            slideIntegral += slideError * deltaTime;
            double slideDerivative = (slideError - slideLastError) / deltaTime;
            double slideOutput = (slideKp * slideError) + (slideKi * slideIntegral) + (slideKd * slideDerivative);

            slideLastError = slideError;
            PIDTimer.reset();

            return slideOutput;
        } else {
            return 0;
        }
    }*/

    private void extendSlide(int slidetarPos){
        slideMotor.setTargetPosition(slidetarPos);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.75);
        while (opModeIsActive() && slideMotor.isBusy()){
            //telemetry.addData("Slide Position",slideMotor.getCurrentPosition());
            //telemetry.update();
        }
        slideMotor.setPower(0);
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


}