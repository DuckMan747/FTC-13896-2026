/*package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.openftc.apriltag.AprilTagDetection;

public class Flywheel {

    private CRServo servo1;
    private CRServo servo2;
    private DcMotorEx launcher;
    private ElapsedTime stateTimer = new ElapsedTime();

    private enum FlywheelState {

        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET_GATE
    }
    private FlywheelState launcherState;

    private double GATE_CLOSE_POWER = -1;
    private double GATE_OPEN_POWER = 1;

    private double GATE_OPEN_TIME = 0.5;
    private double GATE_CLOSE_TIME = 0.5;

    public int shotRemaining = 0;

    private double launcherVelocity = 0;
    private double MIN_LAUNCHER_RPM = 800;
    private double TARGET_LAUNCHER_RPM = 1100;
    private double LAUNCHER_MAX_SPINUP_TIME = 2;

    private init(HardwareMap hardwareMap) {

        servo1 = hardwareMap.get(CRServo.class, "left");
        servo2 = hardwareMap.get(CRServo.class, "right");

        launcher = hardwareMap.get(DcMotorEx.class, "Launcher");

        launcherState = FlywheelState.IDLE;

        launcher.setPower(0);

        servo1.setPower(0);
        servo2.setPower(0);

    }

    private void update(){
        switch (launcherState){
            case IDLE:
                if (shotRemaining > 0){

                    servo1.setPower(GATE_OPEN_POWER);
                    servo2.setPower(GATE_OPEN_POWER);

                    launcher.setPower(TARGET_LAUNCHER_RPM);

                    stateTimer.reset();

                    launcherState = FlywheelState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (launcherVelocity > MIN_LAUNCHER_RPM || stateTimer.seconds() > LAUNCHER_MAX_SPINUP_TIME) {
                    servo1.setPower();

                }
        }
    }
















}
*/



