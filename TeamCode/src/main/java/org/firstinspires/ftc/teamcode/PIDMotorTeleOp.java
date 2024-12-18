
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="PID Motor TeleOp with Joystick & Buttons", group="Linear OpMode")
public class PIDMotorTeleOp extends LinearOpMode {
    DcMotor testMotor;
    DcMotor slideMotor;
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

    // Tuning and timing parameters
    final double POSITION_TOLERANCE = 10; // Acceptable error (tolerance) for reaching the target

    ElapsedTime PIDTimer = new ElapsedTime();

    // User-adjustable controls
    final double POSITION_INCREMENT = 50;  // The amount to increase or decrease the position per button press

    @Override
    public void runOpMode() {
        // Initialize motor
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
        intake = hardwareMap.servo.get("intake");


        waitForStart();

        // Continuous loop to control the motor
        while (opModeIsActive()) {
            // Button-Controlled Positioning
            if (gamepad1.a) {
                targetPosition = -2100;  // Set to a specific target (e.g., 1000 encoder counts)
            }
           /* if (gamepad1.b) {
                targetPosition = -1600;  // Set to a specific target (e.g., -1000 encoder counts)
            }
            if (gamepad1.y) {
                targetPosition = targetPosition - 50;  // Increase position by defined increment
            }
            if (gamepad1.x) {
                targetPosition = targetPosition + 50;  // Decrease position by defined increment
            }
            if (gamepad2.a) {
                slidetarPos = -200;  // Set to a specific target (e.g., 1000 encoder counts)
            }*/
            if (gamepad2.b) {
                slidetarPos = -2200;  // Set to a specific target (e.g., -1000 encoder counts)
            }
            /*if (gamepad2.y) {
                slidetarPos = slidetarPos - 50;  // Increase position by defined increment
            }
            if (gamepad2.x) {
                slidetarPos = slidetarPos + 50;  // Decrease position by defined increment
            }*/
            // Joystick-Controlled Positioning
            // Left joystick controls the target position dynamically
            targetPosition += gamepad1.left_stick_y * 10;  // Scaling factor (10) can be adjusted based on the desired movement speed
            slidetarPos += gamepad2.right_stick_y * 10;

            // Allow user to adjust PID constants with D-pad or other buttons
/*            if (gamepad1.dpad_up) {
                Kp += 0.001;  // Increase Kp
            }
            if (gamepad1.dpad_down) {
                Kp -= 0.001;  // Decrease Kp
            }
            if (gamepad1.dpad_left) {
                Ki += 0.00001;  // Increase Ki
            }
            if (gamepad1.dpad_right) {
                Ki -= 0.00001;  // Decrease Ki
            }
            if (gamepad1.right_bumper) {
                Kd += 0.00001;  // Increase Kd
            }
            if (gamepad1.left_bumper) {
                Kd -= 0.00001;  // Decrease Kd
            }
            if (gamepad2.dpad_up) {
                slideKp += 0.001;  // Increase Kp
            }
            if (gamepad2.dpad_down) {
                slideKp -= 0.001;  // Decrease Kp
            }
            if (gamepad2.dpad_left) {
                slideKi += 0.00001;  // Increase Ki
            }
            if (gamepad2.dpad_right) {
                slideKi -= 0.00001;  // Decrease Ki
            }
            if (gamepad2.right_bumper) {
                slideKd += 0.00001;  // Increase Kd
            }
            if (gamepad2.left_bumper) {
                slideKd -= 0.00001;  // Decrease Kd
            }*/
            if (gamepad2.a){
                wristPos = 0;
            }
            if (gamepad2.b){
                wristPos = 0.75;
            }
            if (gamepad2.y){
                intakePos = 0.75;
            }
            if (gamepad2.right_bumper){
                intakePos = 0;
            }
            if (gamepad2.left_bumper){
                wristPos = 0.25;
            }
            // Call the PID control method to calculate motor power
            double motorPower = pidControl(targetPosition);
            double slidePower = slidepidControl(slidetarPos);

            if (gamepad1.left_bumper){
                targetPosition = -2100;
                if (testMotor.getCurrentPosition() < -1600){
                    slidetarPos = -2200;
                    if (slideMotor.getCurrentPosition() < -2100){
                        wristPos = 0.25;
                    }
                }
            }
            if (gamepad1.right_bumper){
                intakePos = 1;
            }

            motorPower = Math.max(-1, Math.min(1, motorPower));
            slidePower = Math.max(-1, Math.min(1,slidePower));

            // Set the motor power
            testMotor.setPower(motorPower * 0.15);
            slideMotor.setPower(slidePower * 0.1);
            wrist.setPosition(wristPos);
            intake.setPosition(intakePos);

            // Optional: Display motor position and PID constants on the dashboard for debugging
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

            // Sleep to prevent busy waiting and allow time for feedback
            sleep(50);
        }
    }

    // PID control function
    private double pidControl(double targetPosition) {
        double currentPosition = testMotor.getCurrentPosition();
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
        double slideCurPos = slideMotor.getCurrentPosition();
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
        double currentPosition = testMotor.getCurrentPosition();
        return Math.abs(targetPosition - currentPosition) <= POSITION_TOLERANCE;
    }
}
