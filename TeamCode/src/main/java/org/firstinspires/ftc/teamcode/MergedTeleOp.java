package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="MergedTeleOp", group="Comp TeleOp")

public class MergedTeleOp extends LinearOpMode {

    double power = 1;
    double intakePower;
    double wristPower = 1;
    double clawPosition = 0;
    double pivotPower = 0;

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private static final int Max_Position = -2875;
    private static final int Max_Slide = -900;
    private boolean constantPower = false;

    // Declare hardware
    DcMotor linear;
    Servo claw;
    MechanumClass drive = new MechanumClass();
    IMUClass imu = new IMUClass();
    SampleMecanumDrive mecanumDrive;

    @Override
    public void runOpMode() {
        // Initialize hardware
        drive.init(hardwareMap, false);
        imu.initIMU(hardwareMap);
        linear = hardwareMap.get(DcMotor.class, "linearMotor");
        claw = hardwareMap.get(Servo.class, "claw");
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set up Road Runner trajectories
        Pose2d startPos = new Pose2d(49, -60, Math.toRadians(180));
        mecanumDrive.setPoseEstimate(startPos);

        Trajectory traj1 = mecanumDrive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(0, -38), Math.toRadians(0))
                .build();
        Trajectory traj2 = mecanumDrive.trajectoryBuilder(traj1.end())
                .strafeLeft(7.5)
                .build();
        Trajectory traja = mecanumDrive.trajectoryBuilder(traj2.end())
                .strafeRight(7)
                .build();
        Trajectory traj3 = mecanumDrive.trajectoryBuilder(traja.end())
                .splineTo(new Vector2d(49, -58), Math.toRadians(180))
                .build();
        Trajectory traj4 = mecanumDrive.trajectoryBuilder(traj3.end())
                .strafeLeft(3)
                .build();

        // Wait for the game to start
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        mecanumDrive.setPoseEstimate(startPos);
        while (opModeIsActive()) {
            // Driver-controlled code
            double horizontal = -gamepad1.left_stick_y;
            double vertical = -gamepad1.left_stick_x;
            double pivot = -gamepad1.right_stick_x;
            double slidePower = gamepad2.left_stick_x;
            double linearPower = gamepad2.right_stick_x * 0.5;

            int currentPosition = drive.posistion();
            double curSlide = drive.slidePos();
            double linearPos = drive.linearPos();

            if (gamepad2.left_trigger > 0.1) {
                constantPower = !constantPower;
                while (gamepad2.left_trigger > 0.1 && opModeIsActive()) {
                    sleep(500);
                }
            }
            if (constantPower) {
                pivotPower = 0.3;
                sleep(15000);
            } else {
                if (gamepad2.dpad_down) {
                    pivotPower = gamepad2.right_stick_y * 0.5;
                } else if (gamepad2.dpad_up) {
                    pivotPower = gamepad2.right_stick_y * 0.3;
                } else {
                    pivotPower = gamepad2.right_stick_y * 0.05;
                }
            }

            if (gamepad1.left_bumper) {
                power = 0.5;
            } else {
                power = .75;
            }

            if (gamepad2.b) {
                clawPosition = 0.5;
            } else if (gamepad2.y) {
                clawPosition = 0;
            }
            if (gamepad2.left_bumper) {
                wristPower = 0;
            } else if (gamepad2.right_bumper) {
                wristPower = 1;
            }
            else if (gamepad2.dpad_right) {
                wristPower = 0.25;
            }
            else if (gamepad2.dpad_left){
                wristPower =0.15;
            }
            if (gamepad2.a) {
                intakePower = 1;
            } else if (gamepad2.x) {
                intakePower = -1;
            }
            else {
                intakePower = 0;

            }


            if (currentPosition > -1700){
                if (curSlide <-1600){
                    slidePower = 0.1;
                    wristPower = 1;
                }

            }
            if (currentPosition < -2450){
                if (curSlide < -50){
                    wristPower = 1;
                    slidePower = 0.5;
                }
            }


/*            if (currentPosition > Max_Position) {
                if (gamepad2.right_stick_y != 0) {
                    if (gamepad2.dpad_down) {
                        pivotPower = gamepad2.right_stick_y * 0.3;
                    } else if (gamepad2.dpad_up) {
                        pivotPower = gamepad2.right_stick_y * 0.5;
                    } else if (gamepad2.dpad_left) {
                        pivotPower = 0.3;
                    }
                } else {
                    pivotPower = 0;
                }
            } else {
                pivotPower = 0;
                telemetry.addLine("Max position reached!");
                telemetry.update();
            }*/


            drive.teleOP(power, pivot, vertical, horizontal, pivotPower, slidePower, intakePower, wristPower, currentPosition, linearPower, clawPosition, linearPos);

            telemetry.addData("linear", linearPower);
            telemetry.addData("linear Pos", linearPos);
            telemetry.addData("Drive Power", power);
            telemetry.addData("Lift Power", pivotPower);
            telemetry.addData("Slide Power", slidePower);
            telemetry.addData("intakePower", intakePower);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Target Limit", Max_Position);
            telemetry.addData("Slide Pos", curSlide);
            telemetry.addData("Slide Limit", Max_Slide);
            telemetry.addData("constant power", constantPower);
            telemetry.update();
            // Road Runner code
            if (gamepad1.a && gamepad1.b) {

                if (isStopRequested()) return;
                moveArmDown(-61);
                claw.setPosition(0);
                moveArmUp(-1000);
                mecanumDrive.followTrajectory(traj1);
                moveArmUp(-2100);
                mecanumDrive.followTrajectory(traj2);
                moveArmDown(-1100);
                openClaw(0.5, -1000);
            }
            if (gamepad1.x && gamepad1.y) {
                mecanumDrive.followTrajectory(traja);
                mecanumDrive.followTrajectory(traj3);
                mecanumDrive.followTrajectory(traj4);
                moveArmDown(0);
            }
        }
    }

    private void openClaw(double pos, int tarPos) {
        claw.setPosition(pos);
        linear.setTargetPosition(tarPos);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(0.5);

        while (opModeIsActive() && linear.isBusy()) {
            telemetry.addData("ClawPos", pos);
            telemetry.update();
        }
    }

    private void moveArmUp(int tarPos) {
        linear.setTargetPosition(tarPos);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(-0.5);
        claw.setPosition(0);

        while (opModeIsActive() && linear.isBusy()) {
            telemetry.addData("Slide Position", linear.getCurrentPosition());
            telemetry.update();
        }

        linear.setPower(0);
    }

    private void moveArmDown(int tarPos) {
        linear.setTargetPosition(tarPos);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(0.5);
        claw.setPosition(0);

        while (opModeIsActive() && linear.isBusy()) {
            telemetry.addData("Slide Position", linear.getCurrentPosition());
            telemetry.update();
        }
        claw.setPosition(0.5);
    }
}
