package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TopAutoBlue")
public class TopAutoBlue extends OpMode {
    public Follower follower;

    //Timers for our pathing and OpMode switch
    private Timer pathTimer, opModeTimer;

    Mechanism mechanism = new Mechanism();


    public enum PathState {

        // STARTPOS_ENDPOS
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE

        DRIVE_STARTPOS_SHOOTPOS,
        DRIVE_SHOOTPOS_LINEBALLPOS1,
        DRIVE_LINEBALLPOS1_LINECOLLECTPOS1,
        DRIVE_LINECOLLECTPOS1_SHOOTPOS,
        DRIVE_SHOOTPOS_LINEBALLPOS2,
        DRIVE_LINEBALLPOS2_LINECOLLECTPOS2,
        DRIVE_LINECOLLECTPOS2_SHOOTPOS,
        DRIVE_SHOOTPOS_ENDPOS,
        END,
        SHOOT_PRELOADED_ARTIFACTS,
        SHOOT_LINEBALL,
        SHOOT_LINEBALL2



    }

    //Variable for enum
    PathState pathState;
    public final Pose shootPose = new Pose(47.90,95.50, Math.toRadians(138));
    public final Pose startPose = new Pose(21.00, 122.00, Math.toRadians(138));
    public final Pose shootLineControl = new Pose(55.00,86.300);
    public final Pose lineBallPose = new Pose(43.00, 84.00, Math.toRadians(180));
    public final Pose lineCollectControlShoot = new Pose(41.60,87.00);
    public final Pose lineCollectPose = new Pose(27.50,84.00,Math.toRadians(180));
    public final Pose lineBallPose2 = new Pose(43.00,60.00,Math.toRadians(180));
    public final Pose shootLine2Control = new Pose(60.00,69.00);
    public final Pose lineCollect2Pose = new Pose(27.50,60.00,Math.toRadians(180));
    public final Pose lineCollect2Shoot = new Pose(46.00,75.00);
    public final Pose endPose = new Pose(56.038554216867475,105.13734939759036, Math.toRadians(138));
    private PathChain driveStartPOSShootPOS, driveShootPOSLineBallPOS, driveLineBallPOSLineCollectBallPOS, driveLineCollectBallPOSShootPos, driveShootPOSLineBall2POS,driveLineBall2POSLineCollect2POS,driveLineCollect2POSShootPOS, driveShootPOSEndPOS;

    public void buildPath(){
        // building paths from the pathchain before init
        // put in coordinates for starting pose to next pose
        // make sure to put .build() at the end everytime

        // our first path chain being built
        driveStartPOSShootPOS = follower.pathBuilder()
                // BezierLine will move the robot in a constant direction, in this case backwards from the blue thingy
                .addPath(new BezierLine(startPose,shootPose))
                // set a constant heavy heading from startPose to shoot pose
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();

        driveShootPOSLineBallPOS = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose,shootLineControl,lineBallPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), lineBallPose.getHeading())
                .build();

        driveLineBallPOSLineCollectBallPOS = follower.pathBuilder()
                .addPath(new BezierLine(lineBallPose,lineCollectPose))
                .setLinearHeadingInterpolation(lineBallPose.getHeading(), lineCollectPose.getHeading())
                .build();

        driveLineCollectBallPOSShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(lineCollectPose,lineCollectControlShoot,shootPose))
                .setLinearHeadingInterpolation(lineCollectPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPOSLineBall2POS = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose,shootLine2Control,lineBallPose2))
                .setLinearHeadingInterpolation(shootPose.getHeading(),lineBallPose2.getHeading())
                .build();

        driveLineBall2POSLineCollect2POS = follower.pathBuilder()
                .addPath(new BezierLine(lineBallPose2,lineCollect2Pose))
                .setLinearHeadingInterpolation(lineBallPose2.getHeading(), lineCollect2Pose.getHeading())
                .build();

        driveLineCollect2POSShootPOS = follower.pathBuilder()
                .addPath(new BezierCurve(lineCollect2Pose,lineCollect2Shoot,shootPose))
                .setLinearHeadingInterpolation(lineCollect2Pose.getHeading(),shootPose.getHeading())
                .build();

        driveShootPOSEndPOS = follower.pathBuilder()
                // another BezierLine to move us from the white shooting line, to inside of the triangle, this gives us a ranking point
                .addPath(new BezierLine(shootPose,endPose))
                // headings for each position
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdater() {

        switch(pathState) {

            case DRIVE_STARTPOS_SHOOTPOS:
                if (!follower.isBusy()) {
                    follower.followPath(driveStartPOSShootPOS);
                    transitionPathState(PathState.SHOOT_PRELOADED_ARTIFACTS); // resets the called paths timer and transitions into a new state
                    telemetry.addLine("Going to Shoot Position");
                }
                break;

            case SHOOT_PRELOADED_ARTIFACTS:
                //waits a second and brings up the gate and sets the launcher speed to max
                if (!follower.isBusy()) {
                    mechanism.setLauncherSpeed(1);
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.2) {
                        mechanism.closeGate();
                        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.2) {
                            transitionPathState(PathState.DRIVE_SHOOTPOS_LINEBALLPOS1);
                            telemetry.addLine("Shooting Balls");
                        }
                    }
                }
                break;

            case DRIVE_SHOOTPOS_LINEBALLPOS1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    //turns off the launcher and opens the gate
                    mechanism.setLauncherSpeed(0.0);
                    mechanism.openGate();

                //waiting 0.124 seconds for the servo to open the gate
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.624) {
                        //holds the gate
                        mechanism.holdGate();
                        //goes to the LineBallPOS
                        follower.followPath(driveShootPOSLineBallPOS);
                        transitionPathState(PathState.DRIVE_LINEBALLPOS1_LINECOLLECTPOS1); // resets the called paths timer and transitions into a new state
                        telemetry.addLine("Going to 1st Line of Balls");
                    }

                }
                break;

            case DRIVE_LINEBALLPOS1_LINECOLLECTPOS1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    mechanism.setIntakeSpeed(1);

                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                        follower.followPath(driveLineBallPOSLineCollectBallPOS);
                        telemetry.addLine("Collecting Balls");

                        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                            mechanism.closeGate();
                            mechanism.setIntakeSpeed(0);

                            if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.624) {
                                mechanism.holdGate();
                                transitionPathState(PathState.DRIVE_LINECOLLECTPOS1_SHOOTPOS);

                            }
                        }
                    }
                }
                break;

            case DRIVE_LINECOLLECTPOS1_SHOOTPOS:
                if (!follower.isBusy()) {
                    follower.followPath(driveLineCollectBallPOSShootPos);
                    transitionPathState(PathState.SHOOT_LINEBALL);
                    telemetry.addLine("Going to Shoot Position");
                }
                break;

            case SHOOT_LINEBALL:
                //waits a second and brings up the gate and sets the launcher speed to max
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    mechanism.setLauncherSpeed(1);

                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.2) {
                        mechanism.closeGate();

                        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.2) {
                            transitionPathState(PathState.DRIVE_SHOOTPOS_LINEBALLPOS2);
                            telemetry.addLine("Shooting Balls");

                        }
                    }
                }
                break;


            case DRIVE_SHOOTPOS_LINEBALLPOS2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    //turns off the launcher and opens the gate
                    mechanism.setLauncherSpeed(0.0);
                    mechanism.openGate();

                    //waiting 0.124 seconds for the servo to open the gate
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.624) {
                        //holds the gate
                        mechanism.holdGate();
                        //goes to the LineBallPOS
                        follower.followPath(driveShootPOSLineBall2POS);
                        transitionPathState(PathState.DRIVE_LINEBALLPOS2_LINECOLLECTPOS2); // resets the called paths timer and transitions into a new state
                        telemetry.addLine("Going to 2nd Line of Balls");
                    }

                }
                break;

            case DRIVE_LINEBALLPOS2_LINECOLLECTPOS2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    mechanism.setIntakeSpeed(1);

                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                        follower.followPath(driveLineBall2POSLineCollect2POS);
                        telemetry.addLine("Collecting Balls");

                        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                            mechanism.closeGate();
                            mechanism.setIntakeSpeed(0);

                            if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.624) {
                                mechanism.holdGate();
                                transitionPathState(PathState.DRIVE_LINECOLLECTPOS2_SHOOTPOS);

                            }
                        }
                    }
                }
                break;

            case DRIVE_LINECOLLECTPOS2_SHOOTPOS:
                if (!follower.isBusy()) {
                    follower.followPath(driveLineCollect2POSShootPOS);
                    transitionPathState(PathState.SHOOT_LINEBALL2);
                    telemetry.addLine("Going to Shoot Position");

                }
                break;

            case SHOOT_LINEBALL2:
                //waits a second and brings up the gate and sets the launcher speed to max
                if (!follower.isBusy()) {
                    mechanism.setLauncherSpeed(1);
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.2) {
                        mechanism.closeGate();
                        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.2) {
                            transitionPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                            telemetry.addLine("Shooting Balls");
                            mechanism.setLauncherSpeed(0);
                            mechanism.holdGate();
                        }
                    }
                }
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
                if(!follower.isBusy()) {
                    follower.followPath(driveShootPOSEndPOS);
                    transitionPathState(PathState.END);
                    telemetry.addLine("Going to End Position");
                }
                break;

            case END:
                if (follower.isBusy()) {
                    telemetry.addLine("Done all paths");
                }
                break;

            default:
                telemetry.addLine("No State Found");
                break;
        }

    }

    // helper function to reset the timer and transition into a new state
    public void transitionPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        // sets the current path into the drive from the start position to the shoot position
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        mechanism.setIntakeSpeed(0);
        mechanism.setLauncherSpeed(0);
        // init the timers
        pathTimer = new Timer();
        opModeTimer = new Timer();
        // calling the hardwareMap from the Constants
        follower = Constants.createFollower(hardwareMap);
        mechanism.init(hardwareMap);
        // TODO make systems for limelight, servo power injector, drive train, and shooter;

        buildPath();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        //calling the pathState (Found at line 89, allows us to also reset the pathTimer and transition into a new state
        transitionPathState(pathState);
    }

    @Override
    public void loop() {
        //updates pedro pathing coordinates and position
        follower.update();
        statePathUpdater();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("time elapsed", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getX());
        telemetry.addData("heading", follower.getPose().getHeading());



    }
}
