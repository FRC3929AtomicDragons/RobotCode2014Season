/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Robotics
 */
public class PIDTool {
    private double kP, kI, kD;
    private double integral,lastX;
    private double setpoint;
    
    public PIDTool(double kP, double kI, double kD,double setpoint){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.setpoint = setpoint;
        integral = 0.0;
        lastX = 0;
        
    }
    public double computeControl(double x){
        double e = x - setpoint;
        double deriv = x - lastX;
        this.integral += e;
        lastX = x;
        double control = ((kP * e) + (kI* integral) + (kD * deriv));
        return control;
    }
    public void reset(){
        lastX = 0.0;
        integral = 0.0;
        setpoint = 0.0;
        
    }
    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }
}
