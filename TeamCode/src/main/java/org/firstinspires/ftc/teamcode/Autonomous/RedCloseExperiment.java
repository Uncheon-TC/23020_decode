package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

@Autonomous
public class RedCloseExperiment extends OpMode {


    private static final Pose START_POSE =
            new Pose(109, 133, Math.toRadians(90));
    private static final Pose FIRST_POSE =
            new Pose(96, 82, Math.toRadians(0));


    //first line artifact
    private static final Pose FIRST_POSE_1 =
            new Pose(96, 82, Math.toRadians(0));
    private static final Pose SECOND_POSE_1 =
            new Pose(121, 82, Math.toRadians(0));
    private static final Pose THIRD_POSE_1 =
            new Pose(96, 82, Math.toRadians(0));

    //second line artifact
    private static final Pose FIRST_POSE_2 =
            new Pose(96,82, Math.toRadians(0));
    private static final Pose SECOND_POSE_2 =
            new Pose(117, 62, Math.toRadians(0));
    private static final Pose SECOND_CURVE_2 =
            new Pose(92, 56, Math.toRadians(0))
    private static final Pose THIRD_POSE_2 =
            new Pose(96, 82, Math.toRadians(0));

    //third line artifact
    private static final Pose FIRST_POSE_3 =
            new Pose();
    private static final Pose SECOND_POSE_3 =
            new Pose();
    private static final Pose THIRD_POSE_3 =
            new Pose();

    //fourth line artifact
    private static final Pose FIRST_POSE_4 =
            new Pose();
    private static final Pose SECOND_POSE_4 =
            new Pose();
    private static final Pose THIRD_POSE_4=
            new Pose();

    //fifth gate artifact
    private static final Pose FIRST_POSE_5 =
            new Pose();
    private static final Pose SECOND_POSE_5 =
            new Pose();
    private static final Pose THIRD_POSE_5 =
            new Pose();




    String[] options = {
            "1. Grab first line artifacts",
            "2. Grab second line artifacts",
            "3. Grab third line artifacts",
            "4. Grab fourth line artifacts",
            "5. Grab gate artifacts"
    };

    int selected = 0;

    ArrayList<Integer> autonomousRoutes = new ArrayList<>();

    // prevents the menu from moving multiple times from one press
    long lastButtonTime = 0;
    long buttonDelay = 250; // milliseconds



    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

        handleMenu();

        telemetry.addLine("Select route:");
        telemetry.addLine("Starting Position: 103, 133, 90");
        telemetry.addLine("");

        for (int i = 0; i < options.length; i++) {

            if (i == selected) {
                telemetry.addLine("> " + options[i]);
            } else {
                telemetry.addLine("  " + options[i]);
            }
        }

        telemetry.addLine("");
        telemetry.addLine("Custom Autonomous Route:");

        if (autonomousRoutes.isEmpty()) {

            telemetry.addLine("None");

        } else {

            StringBuilder route = new StringBuilder();

            for (int i = 0; i < autonomousRoutes.size(); i++) {

                if (i > 0) {
                    route.append(" -> ");
                }

                route.append(autonomousRoutes.get(i));
            }

            telemetry.addLine(route.toString());
        }

        telemetry.update();
    }

    private void handleMenu() {
        if (System.currentTimeMillis() - lastButtonTime < buttonDelay) {
            return;
        }

        //down

        if (gamepad1.dpad_down) {

            selected++;

            if (selected >= options.length) {
                selected = 0;
            }

            lastButtonTime = System.currentTimeMillis();
        }

        //up

        else if (gamepad1.dpad_up) {

            selected--;

            if (selected < 0) {
                selected = options.length - 1;
            }

            lastButtonTime = System.currentTimeMillis();
        }

        //add

        else if (gamepad1.a) {

            autonomousRoutes.add(selected + 1);

            lastButtonTime = System.currentTimeMillis();
        }

        //remvoe

        else if (gamepad1.b) {

            if (!autonomousRoutes.isEmpty()) {
                autonomousRoutes.remove(autonomousRoutes.size() - 1);
            }

            lastButtonTime = System.currentTimeMillis();
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {

    }
}
