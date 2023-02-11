package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * This is the autonomous state machine, where we make and run the states.
 */

@Autonomous(name = "RedWheel_FarPark")
public class RedWheel_FarPark extends OpMode {

    // INSTANCE VARIABLES
    /**
     * Version of the op-mode file.
     */
    private final double VERSION = 1.0;

    /**
     * The first state to be run.
     */
    private org.firstinspires.ftc.teamcode.auton.State headerState;

    // private IMU imu;

    // METHODS

    /**
     * Sets up all relevant things for the op-mode.
     */
    @Override
    public void init() {

        org.firstinspires.ftc.teamcode.auton.State[] defaultStateSequence = {
                new org.firstinspires.ftc.teamcode.auton.DriveState(5, 0.9, 0, hardwareMap, telemetry),
                new org.firstinspires.ftc.teamcode.auton.DriveState(14, 0.9, 90, hardwareMap, telemetry),
                new org.firstinspires.ftc.teamcode.auton.WheelState(-1, 5, hardwareMap, telemetry),
                new org.firstinspires.ftc.teamcode.auton.DriveState(4, 0.9, 270, hardwareMap, telemetry),
                new org.firstinspires.ftc.teamcode.auton.DriveState(5, 0.9, 180, hardwareMap, telemetry),
                new org.firstinspires.ftc.teamcode.auton.TurnArcState(-90, hardwareMap, telemetry),
                new org.firstinspires.ftc.teamcode.auton.DriveState(100, 0.9, 0, hardwareMap, telemetry),
        };

        // this.imu = IMU.getInstance(IMU.class, hardwareMap);

        headerState = org.firstinspires.ftc.teamcode.auton.StateBuilder.buildStates(defaultStateSequence);
    }

    /**
     * Runs all things related to starting the op-mode.
     */
    @Override
    public void start() {
        // this.imu.setDefaultOrientation();
        this.headerState.start();
    }

    @Override
    public void loop() {
        org.firstinspires.ftc.teamcode.auton.State currentState = headerState.getCurrentState();
        boolean running = currentState != null;

        // Update State
        if (running) {
            currentState.update();
        }

        // Version telemetry.
        telemetry.addLine("Version: " + this.VERSION);
        String status = running ? "RUNNING" : "COMPLETED";
        String currentStateString = running ? currentState.toString() : "None";

        // State telemetry
        telemetry.addLine("CurrentState: " + currentStateString);
        telemetry.addLine("Status: " + status);
        // telemetry.addLine("Orientation: " + this.imu.getOrientation());
    }

    @Override
    public void stop() {
        org.firstinspires.ftc.teamcode.auton.State currentState = headerState.getCurrentState();

        if (currentState != null) {
            currentState.stop();
        }

        // this.imu.close();
    }
}
