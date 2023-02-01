import ilog.concert.IloException;

import java.io.*;

/**
 * @author 574244hn Hoang Thi Khue Nguyen, 562278da Daniël Alblas
 * A main class to run the different p-Mean models from exercises 1-5
 */
public class Main {
    public static void main(String[] args) throws IloException {
        int[][] coordinate_matrix = new int[2][22];
        coordinate_matrix[0][0] = 0;
        coordinate_matrix[1][0] = 0;
        coordinate_matrix[0][21] = 0;
        coordinate_matrix[1][21] = 0;
        //Import excel
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

        // VRP1
        double maximum_charge = 50;
        double time_limit = 235;

        // Initialize and solve for the exercise 1
        VRP model = new VRP(distance_matrix, maximum_charge, time_limit);
        model.solveModel();
    }
}
