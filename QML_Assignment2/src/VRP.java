import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

public class VRP {
    private int nLocations;
    private double[][] d_matrix;
    private double[][] c_matrix;
    private double[][] q_matrix;
    private double[][] t_matrix;
    private double Q;
    private double T;
    private IloCplex cplex;
    private IloNumVar[][] z_matrix;
    private IloNumVar[] rho_vector;
    private IloNumVar[] psi_vector;

    public VRP(double[][] distances, double maxElectricity, double maxTime) throws IloException {
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
        rho_vector = new IloNumVar[nLocations];
        psi_vector = new IloNumVar[nLocations];

        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j < nLocations; j++) {
                z_matrix[i][j] = cplex.boolVar("Arc (" + i + "," + j + ")");
            }
        }

        // Calculate costs, required charge, and time, between locations
        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j < nLocations; j++) {
                c_matrix[i][j] = 1 + d_matrix[i][j];
                q_matrix[i][j] = 10 + Math.pow(d_matrix[i][j], 0.75);
                t_matrix[i][j] = 5 + Math.pow(d_matrix[i][j], 0.9);
            }
        }

        for (int i = 0; i < nLocations; i++) {
            rho_vector[i] = cplex.numVar(0, Q);
            psi_vector[i] = cplex.numVar(0, T);
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

        // Disallows travel to and form the same location
        for (int i = 0; i < nLocations; i++) {
            cplex.addEq(z_matrix[i][i], 0);
        }
        cplex.addEq(z_matrix[0][nLocations - 1], 0);

        // Visit constraints (1b)
        for (int i = 1; i < nLocations - 1; i++) {
            IloNumExpr LHS2 = cplex.constant(0);
            for (int j = 1; j < nLocations; j++) {
                if (j != i) {
                    LHS2 = cplex.sum(LHS2, z_matrix[i][j]);
                }
            }
            cplex.addEq(LHS2, 1);
        }

        // Depart constraints (1c)
        for (int k = 1; k < nLocations - 1; k++) {
            IloNumExpr LHS3 = cplex.constant(0);
            for (int i = 0; i < nLocations - 1; i++) {
                if (i != k) {
                    LHS3 = cplex.sum(LHS3, z_matrix[i][k]);
                }
            }
            cplex.addEq(LHS3, 1);
        }

        // Time and charge constraints to disallow subtours (1d & 1f)
        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j < nLocations; j++) {
                if (i != j) {
                    IloNumExpr RHS_time = cplex.diff(cplex.sum(psi_vector[i], cplex.prod(t_matrix[i][j], z_matrix[i][j])),
                            cplex.prod(T, cplex.diff(1, z_matrix[i][j])));
                    cplex.addGe(psi_vector[j], RHS_time);

                    IloNumExpr RHS_charge = cplex.diff(cplex.sum(rho_vector[i], cplex.prod(q_matrix[i][j], z_matrix[i][j])),
                            cplex.prod(Q, cplex.diff(1, z_matrix[i][j])));
                    cplex.addGe(rho_vector[j], RHS_charge);
                }
            }
        }

        // Constraint to limit time and charge (1e & 1g)
        for (int j = 1; j < nLocations - 1; j++) {
            cplex.addLe(t_matrix[0][j], psi_vector[j]);
            cplex.addLe(psi_vector[j], T - t_matrix[j][nLocations - 1]);
            cplex.addLe(q_matrix[0][j], rho_vector[j]);
            cplex.addLe(rho_vector[j], Q - q_matrix[j][nLocations - 1]);
        }

        cplex.setOut(null);
        cplex.solve();

        // Query the solution
        if (cplex.getStatus() == IloCplex.Status.Optimal) {
            System.out.println("Found optimal solution!");
            System.out.println("Objective = " + cplex.getObjValue());

            int route = 0;
            for (int i = 0; i < nLocations; i++) {
                if (cplex.getValue(z_matrix[0][i]) >= 0.5) {
                    route++;
                    System.out.print("Route " + route + ": 0");
                    int from = 0;
                    int to = i;
                    double totalDistance = 0;
                    double totalCost = 0;
                    double totalTime = 0;
                    double totalCharge = 0;
                    while (to < nLocations) {
                        if (cplex.getValue(z_matrix[from][to]) >= 0.5) {
                            totalDistance += d_matrix[from][to];
                            totalCost += c_matrix[from][to];
                            totalTime += t_matrix[from][to];
                            totalCharge += q_matrix[from][to];
                            //System.out.print("Go from " + from + " to " + to);
                            System.out.print(", " + to);
                            from = to;
                            to = 0;
                        } else {
                            to++;
                        }
                    }
                    System.out.println();
                    System.out.println("Total distance = " + totalDistance);
                    System.out.println("Total cost = " + totalCost);
                    System.out.println("Total time = " + totalTime);
                    System.out.println("Total charge = " + totalCharge);
                    System.out.println();
                }
            }
        } else {
            System.out.println("No optimal solution found");
        }

        // Close the model
        cplex.close();
    }
}
