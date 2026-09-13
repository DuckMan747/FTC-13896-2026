package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Mechanism {

    public CRServo Gservo1;
    public CRServo Gservo2;
    public CRServo Sintake;
    public DcMotor launcher;
    public DcMotor turret;
    public DcMotor intake;

    public void init(HardwareMap hardwareMap) {

        Gservo1 = hardwareMap.get(CRServo.class, "left");
        Gservo2 = hardwareMap.get(CRServo.class, "right");
        Sintake = hardwareMap.get(CRServo.class,"Sintake");

        turret = hardwareMap.get(DcMotorEx.class, "Launcher");
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher = hardwareMap.get(DcMotor.class, "intake");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.get(DcMotor.class, "intake2");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void setIntakeSpeed(double speedIntake) {
        intake.setPower(speedIntake);
        Sintake.setPower(speedIntake);
    }

    public void setLauncherSpeed(double speedLauncher) {
        launcher.setPower(speedLauncher);
    }

    public void closeGate() {
        Gservo1.setPower(1);

    }

    public void openGate(){
        Gservo1.setPower(-1);

    }

    public void holdGate(){
        Gservo2.setPower(1);
    }

    public void test(){
        Gservo2.setPower(-1);
    }
}





