import ilog.concert.IloException;

import java.io.*;

/**
 * @author 574244hn Hoang Thi Khue Nguyen, 562278da Daniël Alblas
 * A main class to run the different p-Mean models from exercises 1-5
 */
public class Main {
    public static void main(String[] args) throws IloException {
        int[][] cost_matrix = new int[33][33];
        //Import excel
        File file = new File("Austin_distances.csv");
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line = br.readLine();
            int i = 0;
            String[] parts = line.split(",");
            int J = parts.length;


            while (line != null) {
                parts = line.split(",");
                for (int j = 0; j < J; j++) {
                    cost_matrix[i][j] = Integer.parseInt(parts[j]);
                }

                i++;
                line = br.readLine();
            }

        } catch (IOException e) {
            e.printStackTrace();
        }
        
        //Import weights
        int[] weights_vector = new int[33];
        File file2 = new File("weight_of_distances.csv");
        try (BufferedReader br2 = new BufferedReader(new FileReader(file2))) {
            String line2 = br2.readLine();
            int i = 0;
            while (line2 != null) {
                weights_vector[i] = Integer.parseInt(line2);
                i++;
                line2 = br2.readLine();
            }

        } catch (IOException e) {
            e.printStackTrace();
        }

        // Small instance
        int[][] small_instance_distances = new int[9][6];
        int[] small_instance_weights = new int[9];
        int[] demandPoints = {8, 16, 19, 23, 24, 26, 28, 32, 33};
        int[] facilityLocations = {8, 9, 11, 14, 23, 27};

        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 6; j++) {
                small_instance_distances[i][j] = cost_matrix[demandPoints[i] - 1][facilityLocations[j] - 1];
            }
            small_instance_weights[i] = weights_vector[demandPoints[i] - 1];
        }

        // Initialize and solve for the small instance (exercise 1)
        pMeanModel model_small = new pMeanModel(small_instance_weights, small_instance_distances, 4);
        model_small.solveModel();

        // Initialize and solve for the full instance (exercise 2)
        pMeanModel model = new pMeanModel(weights_vector, cost_matrix, 4);
        model.solveModel();

        // Initialize and solve for given alpha and delta in the modified p-Mean model (exercise 5)
        for (int a = 70; a <= 100; a++) {
            pMeanModelEx4 modelex4 = new pMeanModelEx4(weights_vector, cost_matrix, 4, 8, a);
            System.out.println(a);
            modelex4.solveModel();
        }
    }
}
