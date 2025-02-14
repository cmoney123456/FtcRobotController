package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Red Right Two", group = "A")
public class redRoadRunnerTwoSpecimenRight extends LinearOpMode {

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


        Pose2d startPos = new Pose2d(24,-64,Math.toRadians(90));

        drive.setPoseEstimate(startPos);





        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                        .splineTo(new Vector2d(8,-38),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                        .strafeLeft(6,SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
        Trajectory trajb = drive.trajectoryBuilder(traj2.end())
                .strafeRight(4,SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(trajb.end())
                .splineTo(new Vector2d(32,-35),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(23,SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(17,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .strafeLeft(42,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .strafeRight(42,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .back(10,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .strafeLeft(49,SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .strafeRight(.5,SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .strafeRight(3)
                .build();
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .splineTo(new Vector2d(4,-38),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
       /* Trajectory trajend = drive.trajectoryBuilder(traj.end())
                        .splineTo(new Vector2d(49,-49),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();*/





        claw.setPosition(0);
        waitForStart();

        if(isStopRequested()) return;

        Thread moveSlideThread = new Thread(new Runnable() {
            public void run() {
                moveArmUp(-2350);
            }
        });
        Thread moveSlideThread2 = new Thread(new Runnable() {
            public void run() {
                moveArmUp(-2350);
            }
        });
        Thread moveSlideDown = new Thread(new Runnable() {
            public void run() {
                moveArmDown(0);
            }
        });
        Thread closeClaw = new Thread(new Runnable() {

            public void run() {
               openClaw(0,-2350);
            }
        });

        moveSlideThread.start();
        drive.followTrajectory(traj1);
        try {
            moveSlideThread.join(); // Wait for the slide movement to complete
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        moveSlideThread2.start();
        drive.followTrajectory(traj2);
        try{
            moveSlideThread2.join();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        moveArmDown(-1200);
        openClaw(0.5,-1000);
        drive.followTrajectory(trajb);
        drive.followTrajectory(traj3);
        //drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        moveSlideDown.start();
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        try {
            moveSlideDown.join(); // Wait for the slide movement to complete
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        drive.followTrajectory(traj10);
        closeClaw.start();
        drive.followTrajectory(traj11);
        drive.followTrajectory(traj12);
        try {
            closeClaw.join();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }








    }

    private void openClaw(double pos, int tarPos) {
        claw.setPosition(pos);
        linear.setTargetPosition(tarPos);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(0.75);


        while (opModeIsActive() && linear.isBusy()){
            telemetry.addData("ClawPos",pos);
            telemetry.update();
        }


    }


    private void moveArmUp(int tarPos) {
        linear.setTargetPosition(tarPos);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(-0.75);
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