package org.firstinspires.ftc.teamcode.Auton;

import static com.sun.tools.doclint.Entity.and;
import static com.sun.tools.javac.jvm.ByteCodes.ret;

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

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroTwelveRedFar extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain FirstIntake;
        public PathChain firstGoal;
        public PathChain SecondIntake;
        public PathChain SecondGoal;
        public PathChain ThirdIntake;
        public PathChain ThirdGoal;

        public Paths(Follower follower) {
            FirstIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(73.951, 16.390),
                                    new Pose(111.610, 42.537),
                                    new Pose(111.024, 34.927),
                                    new Pose(131.707, 35.317)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            firstGoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.707, 35.317),
                                    new Pose(81.171, 19.122),
                                    new Pose(75.902, 11.317)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            SecondIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(75.902, 11.317),
                                    new Pose(100.683, 68.683),
                                    new Pose(106.341, 59.512),
                                    new Pose(134.634, 59.512)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            SecondGoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(134.634, 59.512),
                                    new Pose(78.829, 20.098),
                                    new Pose(75.512, 15.024)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ThirdIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(75.512, 15.024),
                                    new Pose(100.878, 95.610),
                                    new Pose(106.927, 83.317),
                                    new Pose(132.683, 83.707)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ThirdGoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(132.683, 83.707),
                                    new Pose(81.171, 24.585),
                                    new Pose(73.951, 12.098)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {

        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}
