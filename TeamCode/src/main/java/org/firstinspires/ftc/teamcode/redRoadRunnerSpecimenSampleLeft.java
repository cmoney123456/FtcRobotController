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

@Autonomous(name = "Red Left Speci and Sample", group = "Comp")
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
                        .splineTo(new Vector2d(-8,-38),Math.toRadians(0))
                                .build();
        Trajectory traj2 = drive.trajectoryBuilder(startPos)
                        .strafeLeft(6.75)
                                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                        .splineTo(new Vector2d(-36,-48),Math.toRadians(90),SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                        .forward(36,SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                        .strafeLeft(11,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                        .back(48,SampleMecanumDrive.getVelocityConstraint(75, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                        .forward(48,SampleMecanumDrive.getVelocityConstraint(75, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                        .strafeLeft(10,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                        .back(48,SampleMecanumDrive.getVelocityConstraint(75, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                        .forward(48,SampleMecanumDrive.getVelocityConstraint(75, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                        .strafeLeft(10,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                        .back(48,SampleMecanumDrive.getVelocityConstraint(75, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traja = drive.trajectoryBuilder(traj6.end())
                        .splineTo(new Vector2d(-30,-12),Math.toRadians(270))
                                .build();
        Trajectory traje = drive.trajectoryBuilder(traja.end())
                        .strafeLeft(8)
                                .build();





        claw.setPosition(0);
        waitForStart();

        if(isStopRequested()) return;




           // drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
           // drive.followTrajectory(traj3);
            while(opModeIsActive()) {
                raiseArm(-2100, -1850, 0.3, -.5);
            }


    }

    private void raiseArm(double armPos, double slidePos, double wristFlip, double intakeSpeed) {
        double motorPower = pidControl(targetPosition);
        double slidePower = slidepidControl(slidetarPos);

        targetPosition = armPos;
        if (testMotor.getCurrentPosition() < -1600) {
            slidetarPos = slidePos;
            if (slideMotor.getCurrentPosition() < -1800) {
                wristPos = wristFlip;
                sleep(500);
                intakePos = intakeSpeed;
            }
        }


        motorPower = Math.max(-1, Math.min(1, motorPower));
        slidePower = Math.max(-1, Math.min(1, slidePower));

        // Set the motor power
        if (testMotor.getCurrentPosition() < -1600) {
            testMotor.setPower(motorPower * 0.15);
        } else {
            testMotor.setPower(motorPower * 0.15);
        }
        if (slideMotor.getCurrentPosition() < -1500) {
            slideMotor.setPower(slidePower * 0.1);
        } else {
            slideMotor.setPower(slidePower * 0.1);
        }

        wrist.setPosition(wristPos);
        intakeLeft.setPower(-intakePos);
        intakeRight.setPower(intakePos);

        telemetry.addData("Motor Position", testMotor.getCurrentPosition());
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("Slide Motor Position", slideMotor.getCurrentPosition());
        telemetry.addData("Slide Target Position", slidetarPos);
        telemetry.addData("Slide Motor Power", slidePower);
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

    private double slidepidControl(double slidetarPos) {
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