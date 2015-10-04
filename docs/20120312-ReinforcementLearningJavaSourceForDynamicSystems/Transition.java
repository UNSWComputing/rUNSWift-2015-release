package cHEXQSPL;

import java.util.*;

/**
 *
 * @author bernhardhengst
 * 
 * Set of transitions from a state for all actions
 */
public class Transition {
    public float [] s;
    public int a = -1;
    public float r = 0f;
    public float [] nS;
    public float [] delta ; //coarsity of representation
    public float q = 0f;
    public int e = -1; //exit types -1:not specified, 0:not an exit, 1:internal, 2:goal; 3:bad
    public ArrayList fromNeighbours;
    public ArrayList toNeighbours;
    public int iD; 

    
    Transition(int iD, float[] resolution, int dimS){ 
        this.iD = iD; 
        s      = new float[dimS];
        nS     = new float[dimS];
        delta  = new float[dimS]; //coarsity of representation
        fromNeighbours = new ArrayList();
        toNeighbours = new ArrayList();
        System.arraycopy(resolution, 0, delta, 0, dimS);
    } 
    
    public void add(float[] s, int a, float[] nS, float r, int e, boolean bckgrnd){
            System.arraycopy(s, 0, this.s, 0, s.length);
            this.a = a;
            this.r = r;
            System.arraycopy(nS, 0, this.nS, 0, nS.length);
            this.e = e;
    }
}
