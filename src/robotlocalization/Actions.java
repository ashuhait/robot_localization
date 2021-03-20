/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package robotlocalization;

/**
 *
 * @author shado
 */
class Actions {

    private final String robotEvidence; //The evidence the robot senses 
    private final String robotDirection; //The direction the robot moves
    private Map filteredMap; //The filtering after evidence map
    private Map predictionMap; //The prediction after moving map
    
    public String getEvidence() {
        return robotEvidence;
    }

    public String getDirection() {
        return robotDirection;
    }

    public Map getFilteredMap() {
        return filteredMap;
    }

    public void setFilteredMap(Map filteredMap) {
        this.filteredMap = filteredMap;
    }

    public Map getPredictionMap() {
        return predictionMap;
    }

    public void setPredictionMap(Map predictionMap) {
        this.predictionMap = predictionMap;
    }
    
    
    public Actions(String evidence, String direction, Map initialRobotMap) {
        this.robotEvidence = evidence;
        this.robotDirection = direction;
        this.filteredMap = initialRobotMap;
        this.predictionMap = new Map();
    }    
}
