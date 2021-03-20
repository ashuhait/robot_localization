/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package robotlocalization;

import java.math.BigDecimal;
import java.math.MathContext;
import java.math.RoundingMode;


/**
 *
 * @author shado
 */
class Map {
    private final int rowSize = 6;
    private final int colSize = 7;
    private final int precision = 2; 
    private final BigDecimal openSpaces = BigDecimal.valueOf((rowSize * colSize) - 4); //assigns the amount of openspacea by excluding the four obstacles
    private BigDecimal initProbability = BigDecimal.ONE.divide(openSpaces, MathContext.DECIMAL128); //divides one by the openspaces to get the uniform probability
    public BigDecimal map[][]; //2d array that represents the map for the robot
    public String evidenceMap[][]; //2d array that gives us the evidence for each openspace using the map
    public final BigDecimal obstacle = BigDecimal.valueOf(-1.0); //the obstacles are represented by -1
    
    //instantiates the map and evidenceMap and then calls fillWithData
    public Map() {
        this.map = new BigDecimal[rowSize][colSize];
        this.evidenceMap = new String[rowSize][colSize];
        fillWithData();
    } 

    //fills the map with four obstacles in the middle with #### otherwise fills it with initial probability
    //fills the map with the evidence observed at each square
    private void fillWithData() {
        String evidence = "";
        //assigns all the obstacles in the map 
        map[1][1] = obstacle;
        map[1][4] = obstacle;
        map[3][1] = obstacle;
        map[3][4] = obstacle;
        
        for (int row = 0; row < rowSize; row++) {
            for(int column = 0; column < colSize; column++) {
                //if there is a obstacle, skip over it 
                if(map[row][column] == obstacle) continue; 
                map[row][column] = this.initProbability;
                
                //detects if there is an obstacle or not at each of the different directions (west, north, south, east)
                //appends 1 or 0 to the evidence if there is an obstacle or not, respectively 
                if(column - 1 < 0 || map[row][column - 1] == obstacle) evidence += "1";  
                else evidence += "0";
                
                if(row - 1 < 0 || map[row - 1][column] == obstacle) evidence += "1";    
                else evidence += "0";
                
                if(column + 1 >= colSize || map[row][column + 1] == obstacle) evidence += "1";   
                else evidence += "0";
                
                if(row + 1 >= rowSize || map[row + 1][column] == obstacle) evidence += "1";   
                else evidence += "0";
                
                //adds the evidence to the evidenceMap and resets the evidence variable 
                evidenceMap[row][column] = evidence;
                evidence = "";
            }
        }
    }
    
    //prints the map of probabilities
    public void printMap() {
        //keeps only 34 digits for each probability then moves the decimal place to the right using the precision(basically multiplying by 100) to give us the percentage
        //rounds up to two digits after the decimal place
        BigDecimal probability = new BigDecimal("0");
        for (int row = 0; row < rowSize; row++) {
            for (int column = 0; column < colSize; column++) {
                probability = map[row][column];
                if(!probability.equals(obstacle)) {
                probability = probability.round(MathContext.DECIMAL128).movePointRight(this.precision);
                probability = probability.setScale(2, RoundingMode.HALF_UP);
                } 
                System.out.print(probability + " ");
            }
            System.out.println();
        }
        System.out.println("\n");
        
        //assigns all the obstacles in the map
        map[1][1] = obstacle;
        map[1][4] = obstacle;
        map[3][1] = obstacle;
        map[3][4] = obstacle;
    }
    
    public int getRowSize() {
        return rowSize;
    }

    public int getColSize() {
        return colSize;
    }
}
