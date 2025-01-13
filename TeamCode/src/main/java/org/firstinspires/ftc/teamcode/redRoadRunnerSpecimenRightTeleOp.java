package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class redRoadRunnerSpecimenRightTeleOp extends LinearOpMode {

    DcMotor linear;
    Servo claw;


    public void init(HardwareMap hwMap, boolean autoMode){
        linear = hwMap.get(DcMotor.class,"linearMotor");
        claw = hwMap.get(Servo.class,"claw");

    }

    //double wristPower = 0.75;
    //double clawPosition = 1;


    public void runOpMode() {

        init(hardwareMap,false);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPos = new Pose2d(49,-60,Math.toRadians(180));

        drive.setPoseEstimate(startPos);




        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                        .splineTo(new Vector2d(0,-38),Math.toRadians(0))
                                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                        .strafeLeft(7.5)
                                .build();
        Trajectory traja = drive.trajectoryBuilder(traj2.end())
                .strafeRight(7)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traja.end())
                        .splineTo(new Vector2d(49,-58),Math.toRadians((180)))
                                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                        .strafeLeft(3)
                                .build();





        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.a) {
                if (isStopRequested()) return;
                moveArmDown(-61);
                claw.setPosition(0);
                moveArmUp(-1000);
                drive.followTrajectory(traj1);
                moveArmUp(-2100);
                drive.followTrajectory(traj2);
                moveArmDown(-1100);
                openClaw(0.5, -1000);

            }
            if (gamepad1.b){
                drive.followTrajectory(traja);
                drive.followTrajectory(traj3);
                drive.followTrajectory(traj4);
                moveArmDown(0);

            }
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