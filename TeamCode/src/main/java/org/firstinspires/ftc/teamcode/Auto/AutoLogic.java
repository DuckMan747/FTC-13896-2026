package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Sample Paths")
public class AutoLogic extends OpMode {
    private Follower follower;

    //Timers for our pathing and OpMode switch
    private Timer pathTimer, opModeTimer;

    public enum PathState {

        // STARTPOS_ENDPOS
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE

        DRIVE_STARTPOS_SHOOTPOS,
        DRIVE_SHOOTPOS_LINEBALLPOS,
        DRIVE_LINEBALLPOS_LINECOLLECTPOS,
        DRIVE_LINECOLLECTPOS_SHOOTPOS,
        DRIVE_SHOOTPOS_GATEPOS,
        DRIVE_GATEPOS_COLLECTPOS,
        DRIVE_COLLECTPOS_SHOOTPOS,
        DRIVE_SHOOTPOS_ENDPOS,
/*
        SHOOT_PRELOADED_ARTIFACTS,
        SHOOT_LINEBALL,
        SHOOT_GATEBALL

*/

    }

    //Variable for enum
    PathState pathState;
    private final Pose shootPose = new Pose(47.90,95.50, Math.toRadians(138));
    private final Pose startPose = new Pose(21.00, 122.00, Math.toRadians(138));
    private final Pose shootLineControl = new Pose(49.00,86.300);
    private final Pose lineBallPose = new Pose(43.00, 84.00, Math.toRadians(180));
    private final Pose lineCollectControlShoot = new Pose(41.60,87.00);
    private final Pose lineCollectPose = new Pose(30.00,84.00,Math.toRadians(180));
    private final Pose shootControlGate = new Pose(42.00,77.00);
    private final Pose gatePose = new Pose(15.500,70.00,Math.toRadians(180));
    private final Pose control1 = new Pose (20.886,61.908);
    private final Pose control2 = new Pose (14.741,56.703);
    private final Pose collectGate = new Pose(9.00,57.00,Math.toRadians(100));
    private final Pose collectGateControlShoot = new Pose(48.00,79.00);
    private final Pose endPose = new Pose(56.038554216867475,105.13734939759036, Math.toRadians(138));

    private Path shootPreloadedArtifacts, shootLineBall, shootGateBall;

    private PathChain driveStartPOSShootPOS, driveShootPOSLineBallPOS, driveLineBallPOSLineCollectBallPOS, driveLineCollectBallPOSShootPos, driveShootPOSGatePOS, driveGatePOSCollectPOS, driveCollectPOSShootPOS, driveShootPOSEndPOS;

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
                .setConstantHeadingInterpolation(lineBallPose.getHeading())
                .build();

        driveLineCollectBallPOSShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(lineCollectPose,lineCollectControlShoot,shootPose))
                .setLinearHeadingInterpolation(lineCollectPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPOSGatePOS = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose,shootControlGate,gatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),gatePose.getHeading())
                .build();

        driveGatePOSCollectPOS = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose,control1,control2,collectGate))
                .setLinearHeadingInterpolation(gatePose.getHeading(),collectGate.getHeading())
                .build();

        driveCollectPOSShootPOS = follower.pathBuilder()
                .addPath(new BezierCurve(collectGate,collectGateControlShoot,shootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(),shootPose.getHeading())
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
                follower.followPath(driveStartPOSShootPOS, true);
                transitionPathState(PathState.DRIVE_SHOOTPOS_LINEBALLPOS); // resets the called paths timer and transitions into a new state
                break;

            case DRIVE_SHOOTPOS_LINEBALLPOS:
                follower.followPath(driveShootPOSLineBallPOS, true);
                transitionPathState(PathState.DRIVE_LINEBALLPOS_LINECOLLECTPOS); // resets the called paths timer and transitions into a new state
                break;


            case DRIVE_LINEBALLPOS_LINECOLLECTPOS:
                follower.followPath(driveLineBallPOSLineCollectBallPOS,true);
                transitionPathState(PathState.DRIVE_LINECOLLECTPOS_SHOOTPOS);
                break;

            case DRIVE_LINECOLLECTPOS_SHOOTPOS:
                follower.followPath(driveLineCollectBallPOSShootPos);
                transitionPathState(PathState.DRIVE_SHOOTPOS_GATEPOS);
                break;

            case DRIVE_SHOOTPOS_GATEPOS:
                follower.followPath(driveShootPOSGatePOS);
                transitionPathState(PathState.DRIVE_GATEPOS_COLLECTPOS);
                break;

            case DRIVE_GATEPOS_COLLECTPOS:
                follower.followPath(driveGatePOSCollectPOS);
                transitionPathState(PathState.DRIVE_COLLECTPOS_SHOOTPOS);
                break;

            case DRIVE_COLLECTPOS_SHOOTPOS:
                follower.followPath(driveCollectPOSShootPOS);
                transitionPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
                follower.followPath(driveShootPOSEndPOS);
                if (!follower.isBusy()) {
                    telemetry.addLine("Done all paths :D");
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
        // init the timers
        pathTimer = new Timer();
        opModeTimer = new Timer();
        // calling the hardwareMap from the Constants
        follower = Constants.createFollower(hardwareMap);
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
