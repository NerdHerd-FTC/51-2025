package org.firstinspires.ftc.teamcode.AUTOS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Meet_1_Auto")
public class MeetOneAutoTestUsingMeepMeep extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Define start pose
        Pose2d startPose = new Pose2d(-58, -63.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        drive.setPoseEstimate(startPose);

        // Define trajectory sequence
        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-48, -33, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(-58, -33, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(-66, -33, Math.toRadians(120)), Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // Run the trajectory sequence
        drive.followTrajectorySequence(myTrajectory);
    }
}



//package org.firstinspires.ftc.teamcode;

/*import androidx.annotation.NonNull;

 //RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;*/