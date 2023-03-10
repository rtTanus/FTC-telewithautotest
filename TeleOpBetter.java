package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOpBetter", group="OpMode")
public class TeleOpBetter extends OpMode{
    public Servo servoMotor;
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;
    public DcMotor Arm;
    double powServo = null;

    
    @Override
    public void init() {

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setDirection(DcMotor.Direction.FORWARD);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    
    
    public void loop() {
        boolean poderCima = gamepad2.right_bumper;
        boolean poderBaixo = gamepad2.left_bumper;
        boolean poderImovel = gamepad2.x;
        double pow;

        if (poderCima) {
            pow = -0.6;
            Arm.setPower(pow);
        }

        if (poderBaixo) {
            pow = 0.3;
            Arm.setPower(pow);
        }
        if (poderImovel) {
            pow = -0.2;
            Arm.setPower(pow);
        }
        else {
            pow = 0;
            Arm.setPower(pow);
        }
        telemetry.addData("A potencia do motor do sistema linear é de", pow);
        telemetry.addData("Posição do enconder", Arm.getCurrentPosition());

    }

}
