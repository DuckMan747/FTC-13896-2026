package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "test")
public class MotorDirection extends OpMode {

    DcMotor backleft;
    DcMotor frontleft;
    DcMotor Rightfront;
    DcMotor rightback;
    @Override
    public void init() {

        backleft = hardwareMap.get(DcMotor.class, "backleft");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        Rightfront = hardwareMap.get(DcMotor.class, "Rightfront");
        rightback = hardwareMap.get(DcMotor.class, "rightback");

    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            backleft.setPower(1);
            frontleft.setPower(-1);
            Rightfront.setPower(1);
            rightback.setPower(-1);

            telemetry.addData("The ", "backleft motor", "is moving") ;
            // Set Reverse
        }
        if (gamepad1.b) {
            Rightfront.setPower(0);
            telemetry.addData("The ", "frontleft motor", "is moving") ;
            // Set Reverse
        }
        if (gamepad1.y)  {
            Rightfront.setPower(1);
            telemetry.addData("The ", "rightfront motor", "is moving") ;
            // Good
        }
        if (gamepad1.x) {
            Rightfront.setPower(-1);
            telemetry.addData("The ", "rightback motor", "is moving") ;
            // Good
        }

    }
}
