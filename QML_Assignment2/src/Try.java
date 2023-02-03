
import java.io.*;

/**
 * @author 574244hn Hoang Thi Khue Nguyen, 562278da Daniël Alblas
 * A main class to run the different p-Mean models from exercises 1-5
 */
public class Try {
    private int nLocations = 28;
    private double[][] d_matrix;
    private double[][] c_matrix;
    private double[][] t_matrix;
    private double[][] q_matrix;
    private double Q = 50;
    private double T = 235;
    private int nV = 20;
    private int nC = 6;

    public Try() {
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


        int nLocations = 28;
        d_matrix = distance_matrix;


        c_matrix = new double[nLocations][nLocations];
        q_matrix = new double[nLocations][nLocations];
        t_matrix = new double[nLocations][nLocations];

        int[][] z_matrix = new int[nLocations][nLocations];
        //TODO: might be able to use nLocations-1 in the loops instead of below line 47
        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j < nLocations; j++) {
                //z_matrix[i][j] = cplex.boolVar("Arc (" + i + "," + j + ")");
            }
        }

        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j < nLocations; j++) {
                c_matrix[i][j] = 1 + d_matrix[i][j];
                q_matrix[i][j] = 10 + Math.pow(d_matrix[i][j], 0.75);
                t_matrix[i][j] = 5 + Math.pow(d_matrix[i][j], 0.9);
            }
        }

        q_matrix[0][nLocations - 1] = 0;
        t_matrix[0][nLocations - 1] = 0;
        q_matrix[nLocations - 1][0] = 0;
        t_matrix[nLocations - 1][0] = 0;

        c_matrix[0][nLocations - 1] = 0;
        c_matrix[nLocations - 1][0] = 0;
    }

    public double totalCost(int[] route) {
        double cost = 0;
        for (int i = 0; i < route.length-1; i++) {
            cost += c_matrix[route[i]][route[i+1]];
        }
        return cost;
    }

    public double totalTime(int[] route) {
        double time = 0;
        for (int i = 0; i < route.length-1; i++) {
            time += t_matrix[route[i]][route[i+1]];
        }
        return time;
    }

    public void totalCharge(int[] route) {
        double charge = 0;
        for (int i = 0; i < route.length-1; i++) {
            charge += q_matrix[route[i]][route[i+1]];
            if (route[i] >= 21 && route[i] <= 26) {
                System.out.println("Charged used from depot/last station up to station " + (route[i]-20) + ": " + charge);
                charge = 0;
            }
        }
    }

    public void Main(String[] args) {
        Main m = new Main();

        int[] route1 = new int[] {0, 1, 4, 21, 17, 5, 2, 25, 12, 19, 26, 8, 24, 11, 16, 10, 23, 14, 15, 27};
        int[] route2 = new int[] {0, 6, 7, 13, 22, 9, 18, 27};
        int[] route3 = new int[] {0, 20, 3, 27};

        System.out.println("Route 1");
        System.out.println("Cost: " + totalCost(route1));
        System.out.println("Time: " + totalTime(route1));
        totalCharge(route1);

        System.out.println("Route 2");
        System.out.println("Cost: " + totalCost(route2));
        System.out.println("Time: " + totalTime(route2));
        totalCharge(route2);

        System.out.println("Route 3");
        System.out.println("Cost: " + totalCost(route3));
        System.out.println("Time: " + totalTime(route3));
        totalCharge(route3);
    }

//    public static void main(String[] args) throws IloException {
//        int[][] coordinate_matrix = new int[2][22];
//        coordinate_matrix[0][0] = 0;
//        coordinate_matrix[1][0] = 0;
//        coordinate_matrix[0][21] = 0;
//        coordinate_matrix[1][21] = 0;
//        //Import ruins of Rotterdam location coordinates
//        File file = new File("QML_Assignment2/RuinsRotterdam.csv");
//        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
//            String line = br.readLine();
//            int location = 1;
//            String[] parts = line.split(",");
//            int dimensions = parts.length;
//
//
//            while (line != null) {
//                parts = line.split(",");
//                for (int coordinate = 0; coordinate < dimensions; coordinate++) {
//                    coordinate_matrix[coordinate][location] = Integer.parseInt(parts[coordinate]);
//                }
//
//                location++;
//                line = br.readLine();
//            }
//
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//
//        // Creating distance matrix
//        double[][] distance_matrix = new double[22][22];
//        for (int i = 0; i < 22; i++) {
//            for (int j = 0; j < 22; j++) {
//                distance_matrix[i][j] = Math.sqrt(Math.pow((coordinate_matrix[0][i] - coordinate_matrix[0][j]),2) +
//                        Math.pow((coordinate_matrix[1][i] - coordinate_matrix[1][j]),2));
//            }
//        }
//
//        // VRP
//        double maximum_charge = 50;
//        double time_limit = 235;
//
//        // Initialize and solve for the exercise 1
//        VRP model = new VRP(distance_matrix, maximum_charge, time_limit);
//        model.solveModel();
//
//        int[][] coordinate_matrix_temp = new int[2][6];
//        int[][] coordinate_matrix2 = new int[2][28];
//        //Import charging station locations
//        file = new File("QML_Assignment2/ChargingStations.csv");
//        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
//            String line = br.readLine();
//            int location = 0;
//            String[] parts = line.split(",");
//            int dimensions = parts.length;
//
//
//            while (line != null) {
//                parts = line.split(",");
//                for (int coordinate = 0; coordinate < dimensions; coordinate++) {
//                    coordinate_matrix_temp[coordinate][location] = Integer.parseInt(parts[coordinate]);
//                }
//
//                location++;
//                line = br.readLine();
//            }
//
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//
//        for (int i = 0; i < coordinate_matrix.length; i++) {
//            for (int j = 0; j < coordinate_matrix[0].length-1; j++) {
//                coordinate_matrix2[i][j] = coordinate_matrix[i][j];
//            }
//        }
//
//        for (int i = 0; i < coordinate_matrix_temp.length; i++) {
//            for (int j = 0; j < coordinate_matrix_temp[0].length; j++) {
//                coordinate_matrix2[i][j+21] = coordinate_matrix_temp[i][j];
//            }
//        }
//
//        for (int i = 0; i < 2; i++) {
//            coordinate_matrix2[i][27] = coordinate_matrix[i][21];
//        }
//
//        // Creating distance matrix
//        distance_matrix = new double[28][28];
//        for (int i = 0; i < 28; i++) {
//            for (int j = 0; j < 28; j++) {
//                distance_matrix[i][j] = Math.sqrt(Math.pow((coordinate_matrix2[0][i] - coordinate_matrix2[0][j]),2) +
//                        Math.pow((coordinate_matrix2[1][i] - coordinate_matrix2[1][j]),2));
//            }
//        }
//
//        InstFullCharging model2 = new InstFullCharging(distance_matrix, maximum_charge, time_limit, 20, 6);
//        model2.solveModel();
//    }
}
