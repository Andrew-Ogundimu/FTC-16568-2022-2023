package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp1", group="Iterative Opmode")
public class Teleop extends OpMode {
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor xrail = null;
    private float speed = 1;
    private boolean change_x = false;
    private boolean change_y = false;

    private DcMotor lift = null;
    private DcMotor lift2 = null;

    private Servo claw = null;
    private Servo hook = null;
    public void init() {

        fl  = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        xrail = hardwareMap.get(DcMotor.class, "xrail");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        claw = hardwareMap.get(Servo.class, "claw");

        hook = hardwareMap.get(Servo.class, "hook");

        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        xrail.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift2.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status","Initialized");
        telemetry.addData("Current Speed",speed);
    }
    public void init_loop() {

    }
    public float[] rotate(float[] point,float degrees) {
        double d = Math.toRadians((double)degrees);
        return new float[]{(float)(Math.cos(d)*point[0]-Math.sin(d)*point[1]),(float)(Math.cos(d)*point[1]+Math.sin(d)*point[0])};
    }

    @Override
    public void loop() {

        float[] move_vec = new float[]{gamepad1.left_stick_x,gamepad1.left_stick_y};
        float[] r = rotate(move_vec,-45); //rotate the movement vector by 45 degrees
        bl.setPower(r[0]*speed);
        fr.setPower(r[0]*speed);
        fl.setPower(r[1]*speed);
        br.setPower(r[1]*speed);
        if (gamepad1.left_bumper){
            xrail.setPower(1.0f);
        } else if (gamepad1.left_trigger>0) {
            xrail.setPower(-1.0f);
        } else {
            xrail.setPower(0.0f);
        }
        if (gamepad1.right_stick_x!=0) {
            bl.setPower(-gamepad1.right_stick_x*speed);
            fr.setPower(gamepad1.right_stick_x*speed);
            fl.setPower(-gamepad1.right_stick_x*speed);
            br.setPower(gamepad1.right_stick_x*speed);
        }
        if (gamepad1.y!=change_y && gamepad1.y) {
            if (speed<1) {
                speed += 0.1f;
            telemetry.addData("Current Speed",speed);
        } else if (gamepad1.x!=change_x && gamepad1.x) {
            if (speed>0.3) {
                speed -= 0.1f;    
            }
            telemetry.addData("Current Speed",speed);
        }

        if (gamepad1.b) {
            hook.setPosition(0.8);
            telemetry.addData("Servo Position:", hook.getPosition());
        } else if (gamepad1.a){
            hook.setPosition(0);
            telemetry.addData("Servo Position:", hook.getPosition());
        }
        
        change_x = gamepad1.x;
        change_y = gamepad1.y;
    }

}
