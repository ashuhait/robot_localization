/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package robotlocalization;

import java.math.BigDecimal;
import java.math.MathContext;

/**
 *
 * @author shado
 */
public class RobotLocalization {



    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        //transition and emission
        Map initialRobotMap_1 = new Map();
        Map initialRobotMap_2 = new Map();
        Map initialRobotMap_3 = new Map();
        Map initialRobotMap_4 = new Map();
        Map initialRobotMap_5 = new Map();
        int numOfActions = 5;
        Actions sequenceOfActions[] = new Actions[numOfActions];
        
        BigDecimal driftProbability = new BigDecimal(".10");
        BigDecimal moveProbability = new BigDecimal(".80");
        
        //the given transitionMatrix with west, north, south, and east transition probability
        BigDecimal transitionMatrix[][] =  { {moveProbability, driftProbability, BigDecimal.ZERO , driftProbability},
                                             {driftProbability, moveProbability, driftProbability, BigDecimal.ZERO},
                                             {BigDecimal.ZERO, driftProbability, moveProbability, driftProbability},
                                             {driftProbability, BigDecimal.ZERO, driftProbability, moveProbability}
                                      };
        
        //initializes the five actions
        Actions action_1 = new Actions("0000", "N", initialRobotMap_1);
        Actions action_2 = new Actions("1000", "N", initialRobotMap_2);
        Actions action_3 = new Actions("0000", "W", initialRobotMap_3);
        Actions action_4 = new Actions("0101", "W", initialRobotMap_4);
        Actions action_5 = new Actions("1000", "null", initialRobotMap_5);  
        
        
        //adds all the actions to the sequence array 
        sequenceOfActions[0] = action_1;
        sequenceOfActions[1] = action_2;
        sequenceOfActions[2] = action_3;
        sequenceOfActions[3] = action_4;
        sequenceOfActions[4] = action_5;
        
        //prints the initial map then starts simulation
        System.out.println("Initial Location Probabilities ");
        action_1.getFilteredMap().printMap();
        start(sequenceOfActions, transitionMatrix);
    }

    private static void start(Actions[] actions, BigDecimal[][] transitionMatrix) {
        int indexForActions = 0;
        boolean isNormalized = false; //checks to see if the normalizer has be calculated already true if it has been calculated, false otherwise
        BigDecimal probability = BigDecimal.ZERO;
        BigDecimal normalizer = BigDecimal.ZERO;
        while (indexForActions != actions.length) {
            
            if(indexForActions != 0) {
                //copies the previous postDirectionMap to the current postEvidenceMap and then assigns it the current postEvidenceMap
                actions[indexForActions].setFilteredMap(copyMap(actions[indexForActions].getFilteredMap(), actions[indexForActions - 1].getPredictionMap()));
            }
            //filtering;
            FilterProb(actions, indexForActions, probability, normalizer, isNormalized);
            System.out.println("Filtering after Evidence " + "[" + actions[indexForActions].getEvidence() + "]");
            actions[indexForActions].getFilteredMap().printMap();
            
            //Prediction;
            if (!"null".equals(actions[indexForActions].getDirection())) {
                PredictionProb(actions, indexForActions, probability, transitionMatrix);
                System.out.println("Prediction after Action " + actions[indexForActions].getDirection());
                actions[indexForActions].getPredictionMap().printMap();
            }
            indexForActions++;
        }
    }

    private static void FilterProb(Actions[] actions, int indexForActions, BigDecimal probability, BigDecimal normalizer, boolean isNormalizerCalculated) {
        int indexForEvidence = 0;
        int rowSize = actions[0].getFilteredMap().getRowSize();
        int colSize = actions[0].getFilteredMap().getColSize();
        
        //sensing Probabilities
        BigDecimal openSpaceDetectionProb = new BigDecimal("0.85");
        BigDecimal obstacleDetectionProb = new BigDecimal("0.80");
        BigDecimal false_openSpaceDetectionProb = new BigDecimal("0.2");
        BigDecimal false_obstacleDetectionProb = new BigDecimal("0.15");
        for (int row = 0; row < rowSize; row++) {
            for (int column = 0; column < colSize;) {
                //checks if the evidence is null which means our current location is a obstacle
                if (actions[indexForActions].getFilteredMap().evidenceMap[row][column] == null) {
                    column++;
                    continue;
                }
                //assigns the probability from the filteredMap only when the index is at zero
                if (indexForEvidence == 0) {
                    probability = actions[indexForActions].getFilteredMap().map[row][column];
                }
                
                char robotEvidence = actions[indexForActions].getEvidence().charAt(indexForEvidence);  //gets character from the robotEvidence string based on the index
                char mapEvidence = actions[indexForActions].getFilteredMap().evidenceMap[row][column].charAt(indexForEvidence); // gets the character from the mapEvidence based on the index
                
                if (robotEvidence == mapEvidence) {
                    if (robotEvidence == '1') { //if the robot detects the obstacle then we multiply probability by obstacleDectectionProb
                        probability = probability.multiply(obstacleDetectionProb);
                    } else { //if the robot doesnt detect the obstacle then we multiply probability by false_obstacleDectectionProb
                        probability = probability.multiply(openSpaceDetectionProb);
                    }
                } else {
                    if (robotEvidence == '1') { //if the robot mistakes an open space as an obstacle then we multiply probability by openSpaceDectectionProb
                        probability = probability.multiply(false_obstacleDetectionProb);
                    } else { //if the robot detects an open space then we multiply probability by false_openSpaceDectectionProb
                        probability = probability.multiply(false_openSpaceDetectionProb);
                    }
                }
                
                indexForEvidence++;
                
                //if normalizer is calculated and we iterated through the evidenceMap string then we can divide the current probability by the normalizer
                //and assign the probability to the FilteredMap
                if (indexForEvidence > 3 && isNormalizerCalculated) { 
                    probability = probability.divide(normalizer, MathContext.DECIMAL128);
                    actions[indexForActions].getFilteredMap().map[row][column] = probability;
                    indexForEvidence = 0;
                    column++;
                } else if (indexForEvidence > 3) { //if we iterated through the evidenceMap string then we can add the current probability to the normalizer
                    normalizer = normalizer.add(probability);
                    indexForEvidence = 0;
                    column++;
                }
            }
        }
        //base case for the recurisive call
        if(isNormalizerCalculated == true) return; 
        isNormalizerCalculated = true;
        //assigns isNormalizerCalculated to true then does a recursive call to divided every probability by the normalizer then exits the function
        FilterProb(actions, indexForActions, probability, normalizer, isNormalizerCalculated);
    }

    private static void PredictionProb(Actions[] actions, int indexForActions, BigDecimal probability, BigDecimal[][] transitionMatrix) {
        //index for accessing the moving probability
        int westIndex = 0;
        int northIndex = 1;
        int eastIndex = 2;
        int southIndex = 3;
        
        //the translated movement on a 2d array for the four directional movement
        int westMove = -1;
        int northMove = -1;
        int eastMove = 1;
        int southMove = 1;
        
        int rowSize = actions[0].getFilteredMap().getRowSize();
        int colSize = actions[0].getFilteredMap().getColSize();       
        String Direction = actions[indexForActions].getDirection();
        BigDecimal[] transitionRow = null;
        Map map = actions[indexForActions].getFilteredMap();
        
        //assigns the transitionRow based on the given direction 
        switch(Direction) {
            case "W" -> transitionRow = transitionMatrix[westIndex];
            case "N" -> transitionRow = transitionMatrix[northIndex];
            case "E" -> transitionRow = transitionMatrix[eastIndex];
            case "S" -> transitionRow = transitionMatrix[southIndex];
        }
        for (int row = 0; row < rowSize; row++) {
            for (int column = 0; column < colSize; column++) {                
                //calculates the probability for the current space
                probability = prediction(transitionRow, map, row, column, row, column); 
                //calculates the probability for the west space
                probability = probability.add(prediction(transitionRow, map, row, column, row, column + westMove)) ; 
                //calculates the probability for the north space
                probability = probability.add(prediction(transitionRow, map, row, column, row + northMove, column)); 
                //calculates the probability for the east space
                probability = probability.add(prediction(transitionRow, map, row, column, row, column + eastMove)); 
                //calculates the probability for the south space
                probability = probability.add(prediction(transitionRow, map, row, column, row + southMove, column)); 
                //assigns the total probability to the predictionMap
                actions[indexForActions].getPredictionMap().map[row][column] = probability; 
            }
        }
    }

    private static BigDecimal prediction(BigDecimal[] transitionRow, Map map, int row, int col, int postRow, int postCol) {
        //if the current location is on an obstacle or if the current location is outside of the border then return zero
        if(postCol < 0 || postRow < 0 || postRow >= map.getRowSize() || postCol >= map.getColSize() || map.map[postRow][postCol] == map.obstacle) return BigDecimal.ZERO;
        
        //the translated movement on a 2d array for the four directional movement
        int westMove = -1;
        int northMove = -1;
        int eastMove = 1;
        int southMove = 1;
       
        BigDecimal tranProb = null;
        BigDecimal Prob = null;
        BigDecimal totalProb = BigDecimal.ZERO;
        boolean isPositionTheSame = false;
        
        
        for (int index = 0; index < transitionRow.length; index++) {
            
            if (index == 0) {
                if (postCol + westMove < 0 || map.map[postRow][postCol + westMove] == map.obstacle) { //checks if we hit an obstacle or a border
                    if (row == postRow && col == postCol) { //if we bounced back to the same row and col as the original position then assign isPositionTheSame to true 
                        isPositionTheSame = true;
                    }
                } else if(row ==  postRow && col == postCol + westMove) { //if we move to the same row and col as the original position then assign isPositionTheSame to true
                        isPositionTheSame = true;
                    }
            } else if (index == 1) {
                if (postRow + northMove < 0 || map.map[postRow + northMove][postCol] == map.obstacle) { //checks if we hit an obstacle or a border
                    if (row == postRow && col == postCol) { //if we bounced back to the same row and col as the original position then assign isPositionTheSame to true 
                        isPositionTheSame = true;
                    }
                } else if (row ==  postRow + northMove && col == postCol) { //if we move to the same row and col as the original position then assign isPositionTheSame to true
                        isPositionTheSame = true;
                    }
            } else if (index == 2) {
                if (postCol + eastMove >= map.getColSize() || map.map[postRow][postCol + eastMove] == map.obstacle) { //checks if we hit an obstacle or a border
                    if (row == postRow && col == postCol) { //if we bounced back to the same row and col as the original position then assign isPositionTheSame to true 
                        isPositionTheSame = true;
                    }
                } else if (row ==  postRow && col == postCol + eastMove) { //if we move to the same row and col as the original position then assign isPositionTheSame to true
                        isPositionTheSame = true;
                    }
            } else if (index == 3) {
                if (postRow + southMove >= map.getRowSize() || map.map[postRow + southMove][postCol] == map.obstacle) { //checks if we hit an obstacle or a border
                    if (row == postRow && col == postCol) { //if we bounced back to the same row and col as the original position then assign isPositionTheSame to true 
                        isPositionTheSame = true;
                    }
                } else if (row ==  postRow + southMove && col == postCol) { //if we move to the same row and col as the original position then assign isPositionTheSame to true
                        isPositionTheSame = true;
                  }
            }
            //if the position is the same then multiply the transition probability by the FilteredMap probability and add it to the totalProb and reset isPositionTheSame
            if (isPositionTheSame) {
                tranProb = transitionRow[index];
                Prob = map.map[postRow][postCol];
                Prob = Prob.multiply(tranProb);
                totalProb = totalProb.add(Prob);
                isPositionTheSame = false;
            }
        }
        return totalProb;
    }

    private static Map copyMap(Map filteredMap, Map predictionMap) {
        for (int row = 0; row < predictionMap.getRowSize(); row++) {
            for (int column = 0; column < predictionMap.getColSize(); column++) {
               filteredMap.map[row][column] = predictionMap.map[row][column];
            }
        }
        return filteredMap;
    }
}
