package org.firstinspires.ftc.teamcode.Auton;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous red close", group = "Autonomous")
@Configurable // Panels
@Config
public class PedroTwelveRedClose extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(52, -50, Math.toRadians(135)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);


    }


    int path = 1;

    boolean pathRan = false;

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(path); // Update autonomous state machine

        //int path and boolean pathRan are updated inside autonomousPathUpdate();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain firstIntake;
        public PathChain firstGoal;
        public PathChain secondIntake;
        public PathChain secondGoal;
        public PathChain thirdIntake;
        public PathChain thirdGoal;

        public Paths(Follower follower) {
            firstIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(30, -30),
                                    new Pose(15, -20),
                                    new Pose(10, -55)
                                    //new Pose(132.683, 84.293)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            firstGoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(132.683, 84.293),
                                    new Pose(95.415, 91.902),
                                    new Pose(89.171, 84.683)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            secondIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(89.171, 84.683),
                                    new Pose(102.829, 56.390),
                                    new Pose(102.244, 59.122),
                                    new Pose(130.341, 59.512)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            secondGoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.341, 59.512),
                                    new Pose(101.073, 95.220),
                                    new Pose(87.220, 81.951)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            thirdIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(87.220, 81.951),
                                    new Pose(93.854, 27.512),
                                    new Pose(104.780, 35.902),
                                    new Pose(133.854, 35.707)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            thirdGoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.854, 35.707),
                                    new Pose(97.561, 98.732),
                                    new Pose(82.732, 84.878)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public void autonomousPathUpdate(int path) {

            if (path == 1 && !follower.isBusy()) {
                follower.followPath(paths.firstIntake);

                if (!follower.isBusy()) {
                    pathRan = true;
                    path = path + 1;
                }

            }

            //TODO: FIRST, TEST FIRSTINTAKE, THEN TEST REST OF PATHS

            /*else if (path == 2 && !follower.isBusy()) {
                pathRan = false;
                follower.followPath(paths.firstGoal);

                if (!follower.isBusy()) {

                path = path + 1;

                pathRan = true;

                }
            }

            else if (path == 2) {
                pathRan = false;
                follower.followPath(paths.secondIntake);


                if (!follower.isBusy()) {
                    pathRan = true;
                    path = path + 1;
                }
            }*/


            // Add your state machine Here
            // Access paths with paths.pathName
            // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    }
}