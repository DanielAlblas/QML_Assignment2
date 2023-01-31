import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

/**
 * @author 574244hn Hoang Thi Khue Nguyen, 562278da Daniël Alblas
 * An implementation of a deterministic p-Mean model for exercises 1-3
 */
public class pMeanModel {
    private int I;
    private int J;
    private int p;
    private int[][] c_matrix;
    private IloCplex cplex;
    private IloNumVar[][] x_matrix;
    private IloNumVar[] y;

    /**
     * Initializes the p-Mean model
     * @param weights the population of demand points in the form of a weight vector
     * @param distances the distances between point i and location j
     * @param numFacilities the amount of facilities p
     * @throws IloException if any exception is thrown
     */
    public pMeanModel(int[] weights, int[][] distances, int numFacilities) throws IloException {
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
        p = numFacilities;

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
        // Create the objective function
        IloNumExpr obj = cplex.constant(0);
        for (int i = 0; i < I; i++) {
            for (int j = 0; j < J; j++) {
                obj = cplex.sum(obj, cplex.prod(c_matrix[i][j], x_matrix[i][j]));
            }
        }
        cplex.addMinimize(obj);

        // Add the restrictions
        // Constraint 1 (LHS1) ensures each demand point is satisfied by one facility
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

        // Solve the model
        cplex.setOut(null);
        cplex.solve ();

        // Query the solution
        if (cplex.getStatus () == IloCplex.Status.Optimal) {
            System.out.println("Found optimal solution!");
            System.out.println("Objective = " + cplex.getObjValue ());
            for (int j = 0; j < J; j++) {
                if (cplex.getValue(y[j]) >= 0.5) {
                    System.out.println("Location index " + j + ", demand points served: ");
                    for (int i = 0; i < I; i++) {
                        if (cplex.getValue(x_matrix[i][j]) >= 0.5) {
                            System.out.print(i + ",");
                        }
                    }
                    System.out.println();
                }
            }
        }
        else {
            System.out.println("No optimal solution found");
        }
        
        // Close the model
        cplex.close ();
    }
}
