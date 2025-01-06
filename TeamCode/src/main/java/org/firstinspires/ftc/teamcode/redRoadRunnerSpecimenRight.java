package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class redRoadRunnerSpecimenRight extends LinearOpMode {

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


        Pose2d startPos = new Pose2d(12,-64,Math.toRadians(90));

        drive.setPoseEstimate(startPos);




        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                        .splineTo(new Vector2d(0,-38),Math.toRadians(0))
                                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                        .strafeLeft(7)
                                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                        .splineTo(new Vector2d(36,-48),Math.toRadians(90))
                        .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                        .forward(36,SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                        .strafeRight(10,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                        .strafeRight(10,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                        .strafeRight(10,SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                        .back(48,SampleMecanumDrive.getVelocityConstraint(75, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();





        claw.setPosition(0);
        waitForStart();

        if(isStopRequested()) return;


        drive.followTrajectory(traj1);
        moveArmUp(-2000);
        drive.followTrajectory(traj2);
        moveArmDown(-1200);
        openClaw(0.5,-1000);
        drive.followTrajectory(traj3);
        //moveArmDown(0);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
       /* drive.followTrajectory(traj10);
        drive.followTrajectory(traj11);
        drive.followTrajectory(traj12);*/


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