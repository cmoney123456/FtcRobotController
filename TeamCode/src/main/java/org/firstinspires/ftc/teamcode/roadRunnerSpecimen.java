package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class roadRunnerSpecimen extends LinearOpMode {

    DcMotor linear;

    public void init(HardwareMap hwMap, boolean autoMode){
        linear = hwMap.get(DcMotor.class,"linearMotor");
    }

    //double wristPower = 0.75;
    //double clawPosition = 1;


    public void runOpMode() {

        init(hardwareMap,false);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPos = new Pose2d(-36,-64,Math.toRadians(90));

        drive.setPoseEstimate(startPos);




        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                        .splineTo(new Vector2d(0,-38),Math.toRadians(0))
                                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                        .strafeLeft(8)
                                .build();



        waitForStart();

        if(isStopRequested()) return;


        drive.followTrajectory(traj1);
        moveArmUp(-2000);
        drive.followTrajectory(traj2);
        moveArmDown(-1750);



    }

    private void moveArmUp(int tarPos) {
        linear.setTargetPosition(tarPos);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(-0.5);

        while (opModeIsActive()){
            telemetry.addData("Slide Position",linear.getCurrentPosition());
            telemetry.update();
        }

        linear.setPower(0);


    }
    private void moveArmDown(int tarPos){
        linear.setTargetPosition(tarPos);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(0.5);

        while(opModeIsActive()){
            telemetry.addData("Slide Position",linear.getCurrentPosition());
            telemetry.update();
        }
        linear.setPower(0);
    }


}