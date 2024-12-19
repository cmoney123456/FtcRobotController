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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class testPIDAuto extends LinearOpMode {

    DcMotor linear;
    Servo claw;
    DcMotor slide;
    DcMotor lift;
    Servo wrist;
    Servo intake;

    // PID parameters
    double integral = 0;
    double lastError = 0;
    double Kp = 0.001;
    double Ki = 0.00001;
    double Kd = 0.00001;
    double targetPosition = 0;
    double slidetarPos = 0;
    double slideLastError = 0;
    double slideKp = 0.06;
    double slideKi = 0.0006;
    double slideKd = 0.0006;
    double slideIntegral = 0;
    double wristPos = 0.75;
    double intakePos = 0;

    final double POSITION_TOLERANCE = 10;

    ElapsedTime PIDTimer = new ElapsedTime();

    // Reusable PID control classes
    PIDControl liftPID;
    PIDControl slidePID;

    public void init(HardwareMap hwMap, boolean autoMode) {
        linear = hwMap.get(DcMotor.class, "linearMotor");
        claw = hwMap.get(Servo.class, "claw");
        slide = hwMap.get(DcMotor.class, "armMotor");
        lift = hwMap.get(DcMotor.class, "liftMotor");
        wrist = hwMap.get(Servo.class, "wrist");
        intake = hwMap.get(Servo.class, "intake");

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize PID controllers
        liftPID = new PIDControl(Kp, Ki, Kd);
        slidePID = new PIDControl(slideKp, slideKi, slideKd);
    }

    public void runOpMode() {
        init(hardwareMap, false);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPos = new Pose2d(-36, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPos);

        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(-57, -52), Math.toRadians(225))
                .build();

        intake.setPosition(0);
        wrist.setPosition(0.75);
        waitForStart();

        if (isStopRequested()) return;

        // Follow the Road Runner trajectory
        drive.followTrajectory(traj1);
        // Move the arm using PID before moving to the trajectory
        moveArmToPosition(-2100, 0, 0.75, 0);
        moveArmToPosition(-2100, -2100, 0.75, 0);
        openWrist(-2100,-2100,0.75,0);
        openClaw(-2100,-2100,0.75,0);

        // Optionally add more movements here
    }

    private void moveArmToPosition(double targetLiftPos, double targetSlidePos, double wristPos, double intakePos) {
        while (opModeIsActive()) {
            double liftPower = liftPID.calculate(targetLiftPos, lift.getCurrentPosition());
            double slidePower = slidePID.calculate(targetSlidePos, slide.getCurrentPosition());

            liftPower = Math.max(-1, Math.min(1, liftPower));
            slidePower = Math.max(-1, Math.min(1, slidePower));

            lift.setPower(liftPower * 0.15);
            slide.setPower(slidePower * 0.1);

            wrist.setPosition(wristPos);
            intake.setPosition(intakePos);

            telemetry.addData("Lift Position", lift.getCurrentPosition());
            telemetry.addData("Slide Position", slide.getCurrentPosition());
            telemetry.update();

            if (Math.abs(targetLiftPos - lift.getCurrentPosition()) <= POSITION_TOLERANCE &&
                    Math.abs(targetSlidePos - slide.getCurrentPosition()) <= POSITION_TOLERANCE) {
                break;
            }
        }

        // Set final motor powers
        if (targetLiftPos < -1600) {
            lift.setPower(0.01);
        } else {
            lift.setPower(0);
        }
        slide.setPower(0);
    }
    private void openWrist(double targetLiftPos, double targetSlidePos, double wristPos, double intakePos) {
        while (opModeIsActive()) {
            double liftPower = liftPID.calculate(targetLiftPos, lift.getCurrentPosition());
            double slidePower = slidePID.calculate(targetSlidePos, slide.getCurrentPosition());

            liftPower = Math.max(-1, Math.min(1, liftPower));
            slidePower = Math.max(-1, Math.min(1, slidePower));

            lift.setPower(liftPower * 0.15);
            slide.setPower(slidePower * 0.1);

            wrist.setPosition(wristPos);
            intake.setPosition(intakePos);

            telemetry.addData("Lift Position", lift.getCurrentPosition());
            telemetry.addData("Slide Position", slide.getCurrentPosition());
            telemetry.update();

            if (Math.abs(targetLiftPos - lift.getCurrentPosition()) <= POSITION_TOLERANCE &&
                    Math.abs(targetSlidePos - slide.getCurrentPosition()) <= POSITION_TOLERANCE) {
                break;
            }
        }

        // Set final motor powers
        if (targetLiftPos < -1600) {
            lift.setPower(0.01);
        } else {
            lift.setPower(0);
        }
        slide.setPower(0);
    }
    private void openClaw(double targetLiftPos, double targetSlidePos, double wristPos, double intakePos) {
        while (opModeIsActive()) {
            double liftPower = liftPID.calculate(targetLiftPos, lift.getCurrentPosition());
            double slidePower = slidePID.calculate(targetSlidePos, slide.getCurrentPosition());

            liftPower = Math.max(-1, Math.min(1, liftPower));
            slidePower = Math.max(-1, Math.min(1, slidePower));

            lift.setPower(liftPower * 0.15);
            slide.setPower(slidePower * 0.1);

            wrist.setPosition(wristPos);
            intake.setPosition(intakePos);

            telemetry.addData("Lift Position", lift.getCurrentPosition());
            telemetry.addData("Slide Position", slide.getCurrentPosition());
            telemetry.update();

            if (Math.abs(targetLiftPos - lift.getCurrentPosition()) <= POSITION_TOLERANCE &&
                    Math.abs(targetSlidePos - slide.getCurrentPosition()) <= POSITION_TOLERANCE) {
                break;
            }
        }

        // Set final motor powers
        if (targetLiftPos < -1600) {
            lift.setPower(0.01);
        } else {
            lift.setPower(0);
        }
        slide.setPower(0);
    }
    private double pidControl(double targetPosition) {
        double currentPosition = lift.getCurrentPosition();
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
        double slideCurPos = slide.getCurrentPosition();
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

    private boolean isAtTargetPosition(double targetPosition) {
        double currentPosition = lift.getCurrentPosition();
        return Math.abs(targetPosition - currentPosition) <= POSITION_TOLERANCE;
    }

    public class PIDControl {
        private double integral = 0;
        private double lastError = 0;
        private double Kp;
        private double Ki;
        private double Kd;
        private ElapsedTime PIDTimer;

        public PIDControl(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.PIDTimer = new ElapsedTime();
        }

        public double calculate(double targetPosition, double currentPosition) {
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
    }
}
