
package cHEXQSPL;

import java.util.Random;


/**
 *
 * @author bernhardhengst
 * 
 * Accepts state transitions and rewards and builds a model of 
 * the transition function and the reward function
 * using radial basis-like function approximation
 */
public class RLModel{
    //Parameters
    int     dimS    = 2;                        //dimension of continuous state
    int     numA    = 3;                        //number of continuous actions
    int     maxCases = 5000;                    //max tansition cases
    float   alphaQ   = 1.0f;                    //learning rate
    float   gammaQ   = 1.0f;                    //discount factor
    float[] resolution = {0.9f, 0.9f};          //minimum coarsity
    float sharpness = 1f;                       //peakiness of spread    
    public int pCases = 0;                      //pointer to latest case +1
    public Transition [] cases = new Transition[maxCases];  
    public float maxValue;
    public int maxAction;
    int[] rndA = new int[numA];
    Random rnd = new Random();
    
    public RLModel(){
        for (int i=0; i<maxCases; i++) 
            cases[i] = new Transition(i,resolution,dimS);
    }
    
    public void update(float[] s, int a, float[] nS, float r, int e, boolean bckgrnd){
        boolean addCase = true;
        for (int i=0; i<pCases; i++) 
            if(isNeighbourDelta(s,a,e,i,0.9f)) {
                addCase = false;
                break;
            }
        if(addCase) {
            cases[pCases].add(s,a,nS,r,e,bckgrnd);
            updateDelta(cases[pCases]);
            pCases++;
            return ;
        } 
    }
    
    private boolean isNeighbourDelta(float [] s, int a,  int e, int caseId, float spread){ 
        Transition t = cases[caseId];
        if(t.a!=a || t.e!=e) return false;
        for (int d=0; d<dimS; d++) {
            float v1 = s[d];
            float v2 = cases[caseId].s[d];
            float delta = cases[caseId].delta[d];
            if(Math.abs(v1-v2)>spread*delta) return false;
        }
        return true;        
    }
    
    public void solveQ(){
        updateNeighbours(2.0f);
        float totChange = 0;
        int its=0;
        do {
            totChange = 0f;       
            for(int i=0;i<pCases;i++){
                Transition ti = cases[i]; 
                float change = 0f;
                float maxQ = nSneighbourDerivedValue(ti);
                change = ti.r + gammaQ*maxQ - ti.q;
                ti.q += alphaQ*change;
                totChange += Math.abs(change);
            }
            its++;
        } while (totChange>0f); 
    }
    
    public float nSneighbourDerivedValue(Transition t){
        float ret = -100f;
        if(t.e==3) return -100f; //out
        if(t.e==2) return 0f; //goal       
        int numNeighbours = t.toNeighbours.size();      
        float estQneighbour [] = new float[numA];
        float totWeight [] = new float[numA];
        for(int n=0;n<numNeighbours;n++){
            Transition tn = cases[(Integer)t.toNeighbours.get(n)];
            float dist = sharpness*distance(tn.s, t.nS);
            float weight = (float)Math.exp(-dist*dist);
            totWeight[tn.a] += weight;
            estQneighbour[tn.a] += weight*tn.q;     
        }
        for(int nA=0;nA<numA;nA++) {
            if(totWeight[nA]==0f) estQneighbour[nA] = 0f;
            else estQneighbour[nA] /= totWeight[nA];
        }
        for(int nA=0;nA<numA;nA++){            
            if(estQneighbour[nA]>ret) ret = estQneighbour[nA];
        }
        return ret;
    }
    
    public float value(float[] s, float influence){
        float estQneighbour [] = new float[numA];
        float totWeight [] = new float[numA];
        int numNeighbours = 0;
        for(int n=0;n<pCases;n++){ //neighbours
            Transition tn = cases[n];
            if(neighbourCase(s,tn,influence)){
                numNeighbours++;
                float dist = sharpness*distance(tn.s, s);
                float weight = (float)Math.exp(-dist*dist);
                totWeight[tn.a] += weight;
                estQneighbour[tn.a] += weight*tn.q;          
            }
        }
        for(int a=0;a<numA;a++) {
            if(totWeight[a]<=0f) estQneighbour[a] = 0f;
            else estQneighbour[a] /= totWeight[a];
        }
        maxValue = Float.NEGATIVE_INFINITY;
        maxAction = -1;
        for(int a=0;a<numA;a++){
            if(estQneighbour[a]>maxValue) {
                maxValue = estQneighbour[a];
                maxAction = a;
            }
        }      
        return maxValue;  
    }
               
    public void updateNeighbours(float influence){
        for(int i=0;i<pCases;i++){
            Transition ti = cases[i];
            ti.toNeighbours.clear();
            ti.fromNeighbours.clear();
            for(int n=0;n<pCases;n++){ //neighbours
                Transition tn = cases[n];
                if(neighbourCase(ti.nS,tn,influence)) ti.toNeighbours.add(n);
                if(neighbourCase(ti.s, tn, influence)) ti.fromNeighbours.add(n);
            }
        } 
    }
           
    public float distance(float [] s, float [] t) {
        float sum = 0f;
        for (int i=0;i<dimS;i++) {
            float v = s[i]-t[i];
            sum+= v*v;
        }
        return (float)Math.sqrt((double)sum);
    }
    
    private void makeEqual(float [] a, float [] b){
        System.arraycopy(b, 0, a, 0, dimS);
    }
    
    private boolean neighbourCase(float[] s1, Transition t2, float threshold){
        boolean neighbour = true;
        for (int d=0; d<dimS; d++) {
            float v1 = s1[d];
            float v2 = t2.s[d];
            float delta = t2.delta[d];
            if(Math.abs(v1-v2)>threshold*delta) neighbour = false;
        }
        return neighbour;
    }
    
    public void updateDelta(Transition ti){ // revise local coarsity
        for(int d=0;d<dimS;d++){
            float del = Float.MAX_VALUE;
            boolean invokeExitDelta = true;
            for(int a=0;a<numA;a++){
                if(ti.e>0) continue;
                else invokeExitDelta = false;
                if(ti.a==-1) del=0f;    //unused
                float deltaS = Math.abs(ti.s[d]-ti.nS[d]);
                if(deltaS<del) del = deltaS;
            }
            if(invokeExitDelta){ 
                float minDist = Float.MAX_VALUE; 
                int minCaseId = -1;
                for(int n=0;n<pCases;n++){ //neighbours
                    Transition tn = cases[n];
                    float dist = distance(ti.s,tn.s);
                    if(dist<minDist) {
                        minDist = dist;
                        minCaseId = n;
                    }
                }
                if(minCaseId!=-1)
                    System.arraycopy(cases[minCaseId].delta, 0, ti.delta, 0, dimS);
                return;
            }
            if(del<resolution[d])  ti.delta[d] = resolution[d];
            else ti.delta[d] = del;
        }
    }
}
  