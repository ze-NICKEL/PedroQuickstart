package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Twelve Blue Far", group = "Autonomous")
@Config // Panels
public class PedroTwelveBlueFar extends OpMode {

    private Telemetry telemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public static class Paths {

        public PathChain firstintake;
        public PathChain backtogoal;
        public PathChain Secondintake;
        public PathChain secondgoal;
        public PathChain thirdIntake;
        public PathChain thirdGoal;

        public Paths(Follower follower) {
            firstintake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(62.049, 8.976),
                                    new Pose(49.775, 38.567),
                                    new Pose(37.331, 36.078),
                                    new Pose(11.122, 35.707)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            backtogoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.122, 35.707),
                                    new Pose(50.537, 38.634),
                                    new Pose(62.439, 11.512)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Secondintake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(62.439, 11.512),
                                    new Pose(47.610, 70.634),
                                    new Pose(45.659, 58.146),
                                    new Pose(9.756, 60.098)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            secondgoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.756, 60.098),
                                    new Pose(39.024, 42.341),
                                    new Pose(55.415, 36.878),
                                    new Pose(65.171, 13.073)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            thirdIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(65.171, 13.073),
                                    new Pose(43.122, 91.512),
                                    new Pose(40.976, 84.098),
                                    new Pose(12.488, 84.683)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            thirdGoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(12.488, 84.683),
                                    new Pose(12.683, 49.366),
                                    new Pose(50.927, 54.634),
                                    new Pose(64.585, 10.146)
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