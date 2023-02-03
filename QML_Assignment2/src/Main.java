import ilog.concert.IloException;

import java.io.*;
import java.util.ArrayList;

/**
 * @author 574244hn Hoang Thi Khue Nguyen, 562278da Daniël Alblas
 * A main class to run the different p-Mean models from exercises 1-5
 */
public class Main {
    public static void main(String[] args) throws IloException {
//        Try m = new Try();
        int[][] coordinate_matrix = new int[2][22];
        coordinate_matrix[0][0] = 0;
        coordinate_matrix[1][0] = 0;
        coordinate_matrix[0][21] = 0;
        coordinate_matrix[1][21] = 0;
        //Import ruins of Rotterdam location coordinates
        File file = new File("QML_Assignment2/RuinsRotterdam.csv");
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line = br.readLine();
            int location = 1;
            String[] parts = line.split(",");
            int dimensions = parts.length;


            while (line != null) {
                parts = line.split(",");
                for (int coordinate = 0; coordinate < dimensions; coordinate++) {
                    coordinate_matrix[coordinate][location] = Integer.parseInt(parts[coordinate]);
                }

                location++;
                line = br.readLine();
            }

        } catch (IOException e) {
            e.printStackTrace();
        }

        // Creating distance matrix
        double[][] distance_matrix = new double[22][22];
        for (int i = 0; i < 22; i++) {
            for (int j = 0; j < 22; j++) {
                distance_matrix[i][j] = Math.sqrt(Math.pow((coordinate_matrix[0][i] - coordinate_matrix[0][j]),2) +
                        Math.pow((coordinate_matrix[1][i] - coordinate_matrix[1][j]),2));
            }
        }

        // VRP
        double maximum_charge = 50;
        double time_limit = 235;

//        // Initialize and solve for the exercise 1
//        VRP model = new VRP(distance_matrix, maximum_charge, time_limit);
//        model.solveModel();

        int[][] coordinate_matrix_temp = new int[2][6];
        int[][] coordinate_matrix2 = new int[2][28];
        //Import charging station locations
        file = new File("QML_Assignment2/ChargingStations.csv");
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line = br.readLine();
            int location = 0;
            String[] parts = line.split(",");
            int dimensions = parts.length;


            while (line != null) {
                parts = line.split(",");
                for (int coordinate = 0; coordinate < dimensions; coordinate++) {
                    coordinate_matrix_temp[coordinate][location] = Integer.parseInt(parts[coordinate]);
                }

                location++;
                line = br.readLine();
            }

        } catch (IOException e) {
            e.printStackTrace();
        }

        for (int i = 0; i < coordinate_matrix.length; i++) {
            for (int j = 0; j < coordinate_matrix[0].length-1; j++) {
                coordinate_matrix2[i][j] = coordinate_matrix[i][j];
            }
        }

        for (int i = 0; i < coordinate_matrix_temp.length; i++) {
            for (int j = 0; j < coordinate_matrix_temp[0].length; j++) {
                coordinate_matrix2[i][j+21] = coordinate_matrix_temp[i][j];
            }
        }

        for (int i = 0; i < 2; i++) {
            coordinate_matrix2[i][27] = coordinate_matrix[i][21];
        }

        // Creating distance matrix
        distance_matrix = new double[28][28];
        for (int i = 0; i < 28; i++) {
            for (int j = 0; j < 28; j++) {
                distance_matrix[i][j] = Math.sqrt(Math.pow((coordinate_matrix2[0][i] - coordinate_matrix2[0][j]),2) +
                        Math.pow((coordinate_matrix2[1][i] - coordinate_matrix2[1][j]),2));
            }
        }

        ChargingVRP model2 = new ChargingVRP(distance_matrix, maximum_charge, time_limit, 20, 6, new ArrayList<>(), new ArrayList<>());
        model2.solveModel();
    }
}
