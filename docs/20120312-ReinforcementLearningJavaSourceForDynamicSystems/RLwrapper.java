package cHEXQSPL;

import bernhard.simpleCart.*;
import java.util.Random;

/**
 *
 * @author bernhardhengst
 */
public class RLwrapper {    
    public RLModel m;                           //model
    boolean physicsBackground = true;           //use system model to start
    float r = 0;                                //default reward
    public float exploration = 0.0f;            //epsion-greed RL exploration
    float max = Float.MAX_VALUE;
    public float [] lastS = {max,max};          //signal not set yet
    Random rnd = new Random();
    int action;
    public float goal [] = {0f, 0f};
    
    public RLwrapper(){
        m = new RLModel();
        if(physicsBackground) {
            buildModule(); 
            physicsBackground = false;           
            m.solveQ();
        }
    }
    
    private void buildModule(){
        float dt = 1f/6f;                       //set time step eg 1/100 
        float x,y; int a;                       // state and action variables
        for(x=-20.5f;x<20.5f;x+=0.1f) for(y=-20.5f;y<20.5f;y+=0.1f)for(a=0;a<3;a++){
            float [] s = {x,y};                 //state vector
            if(cartOutOfBounds(s)) continue;    //can't start out of bounds 
            // next state using physics process model
            float[] nS = {x + y*dt+0.5f*9.6f*(a-1)*dt*dt, y + 9.6f*(a-1)*dt};
            int e = 0; r = -1f;                 // e = exit type
            if(goalReached(nS,2.0f)) {e=2; r=0f;}
            if(cartOutOfBounds(nS))  {e=3; r=0f;}
            m.update(s,a,nS,r,e,physicsBackground);
        }
    }
        
    public int update(float[] s){        
        if(goalReached(s,2f) || cartOutOfBounds(s)) return -1;        
        m.value(s,2.0f);
        action = m.maxAction;
        return action;
    }

    public boolean goalReached(float state [], float delta){
        if(Math.abs(goal[0]-state[0])<delta && Math.abs(goal[1]-state[1])<delta) return true;
        return false;
    }
  
    public boolean cartOutOfBounds(float [] s){
        if(Math.abs(s[0])>20f || Math.abs(s[1])>20f) return true;
        return false;
    }
    
}
