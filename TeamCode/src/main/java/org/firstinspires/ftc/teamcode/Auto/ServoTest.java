package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;

import java.util.Timer;
@TeleOp
public class ServoTest extends OpMode {
    Mechanism mechanism = new Mechanism();

    private Timer hello;

    @Override
    public void init() {
        mechanism.init(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            mechanism.openGate();
        }

        if (gamepad1.b){
            mechanism.closeGate();
        }

        if (gamepad1.x){
            mechanism.holdGate();
        }

        if (gamepad1.y){
            mechanism.test();
        }


    }
}
