import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

/**
 * @author 574244hn Hoang Thi Khue Nguyen, 562278da Daniël Alblas
 * An implementation of a deterministic p-Mean model with additional distance constraint for exercises 4-5
 */
public class pMeanModelEx4 {
    private int I;
    private int J;
    private int p;
    private int[][] c_matrix;
    private int[][] z_matrix;
    private int[] weight_vector;
    private double delta;
    private double alpha;
    private IloCplex cplex;
    private IloNumVar[][] x_matrix;
    private IloNumVar[] y;

    /**
     * Initializes the modified p-Mean model
     * @param weights the population of demand points in the form of a weight vector
     * @param distances the distances between point i and location j
     * @param numFacilities the amount of facilities p
     * @param d the maximum distance that a% of the population can be served by the chosen facilities
     * @param a the percentage for which the maximum distance can be served by the chosen facilities
     * @throws IloException if any exception is thrown
     */
    public pMeanModelEx4(int[] weights, int[][] distances, int numFacilities, double d, double a) throws IloException {
        // Initialize the cplex solver
        cplex = new IloCplex();
        // Total number of demand points (index i)
        I = distances.length;
        // Total number of facility locations (index j)
        J = distances[0].length;

        // Creating the cost matrix, in which each entry (i,j) is a product of weight at demand point i and the distance between i and facility j
        c_matrix = new int[I][J];
        for (int i = 0; i < I; i++) {
            for (int j = 0; j < J; j++) {
                c_matrix[i][j] = weights[i] * distances[i][j];
            }
        }

        weight_vector = weights;
        p = numFacilities;
        delta = d;
        alpha = a;

        // Creating the indicator matrix equal to 1 if the distance(i,j) <= delta and otherwise 0
        z_matrix = new int[I][J];

        for (int i = 0; i < I; i++) {
            for (int j = 0; j < J; j++) {
                if (distances[i][j] <= delta) {
                    z_matrix[i][j] = 1;
                } else {
                    z_matrix[i][j] = 0;
                }
            }
        }

        // Creating the decision variables: matrix x and vector y
        x_matrix = new IloNumVar[I][J];
        y = new IloNumVar[I];

        for (int i = 0; i < I; i++) {
            for (int j = 0; j < J; j++) {
                x_matrix[i][j] = cplex.boolVar("x_matrix" + i + "," + j);
            }

            y[i] = cplex.boolVar("y" + i);
        }
    }

    /**
     * Solves the model
     * @throws IloException if any exception is thrown
     */
    public void solveModel() throws IloException {

        // Create the objective
        IloNumExpr obj = cplex.constant(0);
        for (int i = 0; i < I; i++) {
            for (int j = 0; j < J; j++) {
                obj = cplex.sum(obj, cplex.prod(c_matrix[i][j], x_matrix[i][j]));
            }
        }
        cplex.addMinimize(obj);

        // Add the restrictions
        // Constraint 1 (LHS1) ensures each demand point is satisfied by one facilit
        for (int i = 0; i < I; i++) {
            IloNumExpr LHS1 = cplex.constant(0);
            for (int j = 0; j < J; j++) {
                LHS1 = cplex.sum(LHS1, x_matrix[i][j]);
            }
            cplex.addEq(LHS1, 1);
        }

        // Constraint 2 (LHS2) restricts the placement or allocation of facilities to exactly p
        IloNumExpr LHS2 = cplex.constant(0);
        for (int j = 0; j < J; j++) {
            for (int i = 0; i < I; i++) {
                // These constraints maintain that any such assignment must be made to only those sites that have been selected for a facility
                cplex.addLe(x_matrix[i][j], y[j]);
            }
            LHS2 = cplex.sum(LHS2, y[j]);
        }
        cplex.addEq(LHS2, p);

        // Additional constraint which further restricts the model based on distance, alpha and delta
        IloNumExpr LHS3 = cplex.constant(0);
        for (int i = 0; i < I; i++) {
            for (int j = 0; j < J; j++) {
                LHS3 = cplex.sum(LHS3, cplex.prod(cplex.prod(x_matrix[i][j], z_matrix[i][j]), weight_vector[i]));
            }
        }
        int totalPopulation = 0;
        for (int i = 0; i < weight_vector.length; i++) {
            totalPopulation += weight_vector[i];
        }
        cplex.addGe(LHS3, (double) alpha / 100 * totalPopulation);

        // Solve the model
        cplex.setOut(null);
        cplex.solve ();

        // Query the solution
        if (cplex.getStatus () == IloCplex.Status.Optimal) {
            System.out.println("Found optimal solution!");
            System.out.println("Objective = " + cplex.getObjValue());
        }
        else {
            System.out.println("No optimal solution found");
        }
        // Close the model
        cplex.close ();
    }
}
