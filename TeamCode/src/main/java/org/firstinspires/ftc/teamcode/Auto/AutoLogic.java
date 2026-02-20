package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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
        SHOOT_PRELOADED_ARTIFACTS,
        DRIVE_SHOOTPOS_LINEBALLPOS,
        DRIVE_BALLPOS_LINECOLLECTPOS,
        DRIVE_LINECOLLECTPOS_SHOOTPOS,
        DRIVE_SHOOTPOS_GATEPOS,
        DRIVE_GATEPOS_COLLECTPOS,
        DRIVE_COLLECTPOS_SHOOTPOS,
        DRIVE_SHOOTPOS_ENDPOS


    }

    //Variable for enum
    PathState pathState;

    private final Pose startPose = new Pose(47.84578313, 95.50843373, Math.toRadians(138));

    private final Pose shootPose = new Pose(47.722,95.508433, Math.toRadians(138));

    private final Pose ballPose = new Pose(41, 83.69397590361447, Math.toRadians(180));

    private final Pose endPose = new Pose(56.038554216867475,105.13734939759036, Math.toRadians(138));

    private PathChain driveStartPOSShootPOS,driveShootPOSBallPOS, driveShootPOSEndPOS;

    public void buildPath(){
        // building paths from the pathchain before init
        // put in coordinates for starting pose to next pose
        // make sure to put .build() at the end everytime

        // our first path chain being built
        driveStartPOSShootPOS = follower.pathBuilder()
                // BezierLine will move the robot in a constant direction, in this case backwards from the blue thingy
                .addPath(new BezierLine(startPose,shootPose))
                // headings for both of our paths
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPOSEndPOS = follower.pathBuilder()
                // another BezierLine to move us from the white shooting line, to inside of the triangle, this gives us a ranking point
                .addPath(new BezierLine(shootPose,endPose))
                // headings for each position
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
        driveShootPOSBallPOS = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose,ballPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), ballPose.getHeading())
                .build();
    }

    public void statePathUpdater() {
        switch(pathState) {

            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPOSShootPOS, true);
                transitionPathState(PathState.SHOOT_PRELOADED_ARTIFACTS); // resets the called paths timer and transitions into a new state
                break;

            case SHOOT_PRELOADED_ARTIFACTS:
                // check if follower is done moving on it's path and checks if 5 seconds have passed
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    //TODO add in shooting logic
                    follower.followPath(driveShootPOSBallPOS, true);
                    transitionPathState(PathState.DRIVE_SHOOTPOS_LINEBALLPOS); // resets the called paths timer and transitions into a new state
                    telemetry.addLine("Shooting balls");

                }
                break;

            case DRIVE_SHOOTPOS_LINEBALLPOS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >3) {
                    follower.followPath(driveShootPOSEndPOS, true);
                    transitionPathState(PathState.DRIVE_SHOOTPOS_ENDPOS); // resets the called paths timer and transitions into a new state
                    telemetry.addLine("Shooting balls");
                }
                break;

                //TODO add in a second shoot balls

            case DRIVE_SHOOTPOS_ENDPOS:
                //done with paths :D
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
