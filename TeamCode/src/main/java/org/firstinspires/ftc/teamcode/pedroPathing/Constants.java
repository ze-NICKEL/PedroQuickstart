package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()

            // 1 TODO: 1 GET MASS OF BOT(IN KILOGRAMS) by finding weight(in pounds), then converting to kilograms
            .mass(0)

            //TODO: 5 Run ForwardZeroPowerAccelerationTuner after finding max x and y velocity
            .forwardZeroPowerAcceleration(0)

            //TODO: 6 Run LateralZeroPowerAccelerationTuner to find side deceleration rate
            .lateralZeroPowerAcceleration(0);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            //4 TODO: 4 RUN Forward and LateralVelocityTuner and input end result(max directional velocity)
            .xVelocity(0)
            .yVelocity(0);

    public static PinpointConstants localizerConstants = new PinpointConstants()

            // 2 TODO:  2 MEASURE DISTANCE BETWEEN STRAFE POD AND FORWARD POD
            .forwardPodY(-5)
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

            // 3 TODO: 3  FIND ENCODER DIRECTION BY RUNNING LOCALIZATION TEST(SEE IF ROBOT MOVEMENT CORRELATES TO X/Y VALUES)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);





}
