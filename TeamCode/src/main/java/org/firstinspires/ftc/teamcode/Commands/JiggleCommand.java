package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.utils.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;


public class JiggleCommand extends CommandBase {
    private  final double JIGGLE_POWER = 0.2;
     private Timer jiggleTimer = new Timer();
     private Timer jigTimer = new Timer();
     private int jiggleTime;
     private final int jigTime = 200;
     private boolean movingLeft;
     private boolean firstJig;
     private BiobuzzRobot robot;

     public JiggleCommand(BiobuzzRobot robot,int jiggleTimeMs){
         this.jiggleTime = jiggleTimeMs;
         this.robot = robot;
     }

    @Override
    public void initialize(){
       movingLeft = true;
        jiggleTimer.reset();
        jigTimer.reset();
        robot.drive.manual(0,JIGGLE_POWER,0);
        firstJig = true;
    }

    @Override
    public void execute(){
        // cycling between left and right
        if(firstJig && (jigTimer.milliseconds() >= (jigTime/2))){
            robot.drive.manual(0, -JIGGLE_POWER, 0);
            movingLeft = false;
            firstJig = false;
            jigTimer.reset();
        } else if (jigTimer.milliseconds() >= jigTime){
            robot.drive.manual(0, movingLeft ? -JIGGLE_POWER : JIGGLE_POWER, 0);
            movingLeft = !movingLeft;
            jigTimer.reset();
        }
    }

    @Override
    public boolean isFinished(){
        return (jiggleTimer.milliseconds() >= jiggleTime);
    }

    @Override
    public void end(boolean interrupted){
        robot.drive.stop();
    }











}
