import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

public class VRP1 {
    private int nLocations;
    private double[][] d_matrix;
    private double[][] c_matrix;
    private double[][] q_matrix;
    private double[][] t_matrix;
    private double Q;
    private double T;
    private IloCplex cplex;
    private IloNumVar[][] z_matrix;
    private IloNumVar[] q_vector;
    private IloNumVar[] t_vector;

    public VRP1(double[][] distances, double maxElectricity, double maxTime) throws IloException {
        // Initialize the cplex solver
        cplex = new IloCplex();

        nLocations = distances.length;
        d_matrix = distances;
        Q = maxElectricity;
        T = maxTime;

        c_matrix = new double[nLocations][nLocations];
        q_matrix = new double[nLocations][nLocations];
        t_matrix = new double[nLocations][nLocations];

        // Creating the decision variables: matrix x and vector y
        z_matrix = new IloNumVar[nLocations][nLocations];
        q_vector = new IloNumVar[nLocations];
        t_vector = new IloNumVar[nLocations];

        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j < nLocations; j++) {
                c_matrix[i][j] = 1 + d_matrix[i][j];
                q_matrix[i][j] = 10 + Math.pow(d_matrix[i][j], 0.75);
                t_matrix[i][j] = 5 + Math.pow(d_matrix[i][j], 0.9);
                z_matrix[i][j] = cplex.boolVar("Arc (" + i + "," + j + ")");
            }
            q_vector[i] = cplex.numVar(0, Q, "Electric used at location " + i);
            t_vector[i] = cplex.numVar(0, T, "Time at location " + i);
        }
    }

    public void solveModel() throws IloException {
        // Create the objective function
        IloNumExpr obj = cplex.constant(0);
        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j < nLocations; j++) {
                obj = cplex.sum(obj, cplex.prod(c_matrix[i][j], z_matrix[i][j]));
            }
        }
        cplex.addMinimize(obj);

        // Add the restrictions
        // Constraints 1 and 2 (LHS1 and LHS) ensures every location (except the depot) is entered and left once
        for (int j = 1; j < nLocations; j++) {
            IloNumExpr LHS1 = cplex.constant(0);
            IloNumExpr LHS2 = cplex.constant(0);
            for (int i = 0; i < nLocations; i++) {
                LHS1 = cplex.sum(LHS1, z_matrix[i][j]);
                LHS2 = cplex.sum(LHS2, z_matrix[j][i]);
            }
            cplex.addEq(LHS1, 1);
            cplex.addEq(LHS2, 1);
        }



        for (int i = 0; i < nLocations; i++) {
            for (int j = 1; j < nLocations; j++) {
                cplex.addLe(cplex.prod(z_matrix[i][j], (q_matrix[i][j] + q_matrix[j][0])), q_vector[i]);
                cplex.addLe(cplex.prod(z_matrix[i][j], (t_matrix[i][j] + t_matrix[j][0])), t_vector[i]);
            }
        }

        // Solve the model
        cplex.setOut(null);
        cplex.solve ();

        // Query the solution
        if (cplex.getStatus () == IloCplex.Status.Optimal) {
            System.out.println("Found optimal solution!");
            System.out.println("Objective = " + cplex.getObjValue ());
            for (int j = 0; j < nLocations; j++) {
                    for (int i = 0; i < nLocations; i++) {
                        if (cplex.getValue(z_matrix[i][j]) >= 0.5) {
                            System.out.println("Location " + i + ", " + j);
                            System.out.println("Charge:" + q_vector[i] + ", time: " + t_vector[i]);
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
