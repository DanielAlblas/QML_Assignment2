import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

public class InstFullCharging {
    private int nLocations;
    private int nV;
    private int nC;
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

    public InstFullCharging(double[][] distances, double maxElectricity, double maxTime, int numLocations, int numChargeStations) throws IloException {
        // Initialize the cplex solver
        cplex = new IloCplex();

        nLocations = distances.length;
        d_matrix = distances;
        Q = maxElectricity;
        T = maxTime;
        nV = numLocations;
        nC = numChargeStations;

        c_matrix = new double[nLocations][nLocations];
        q_matrix = new double[nLocations][nLocations];
        t_matrix = new double[nLocations][nLocations];

        // Creating the decision variables: matrix x and vector y
        z_matrix = new IloNumVar[nLocations][nLocations];
        q_vector = new IloNumVar[nLocations];
        t_vector = new IloNumVar[nLocations];

        //TODO: might be able to use nLocations-1 in the loops instead of below line 47
        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j < nLocations; j++) {
                z_matrix[i][j] = cplex.boolVar("Arc (" + i + "," + j + ")");
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

        for (int i = 0; i < nLocations; i++) {
            q_vector[i] = cplex.numVar(0, Q);
            t_vector[i] = cplex.numVar(0, T);
        }

        c_matrix[0][nLocations - 1] = 0;
        c_matrix[nLocations - 1][0] = 0;
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

        // 2b
        for (int i = 1; i <= nV; i++) {
            IloNumExpr LHS2b = cplex.constant(0);
            for (int j = 1; j <= nV ; j++) {
                if (j != i) {
                    LHS2b = cplex.sum(LHS2b, z_matrix[i][j]);
                }
            }
            // Including also the arc goes to the ending depot
            LHS2b = cplex.sum(LHS2b, z_matrix[i][nV+nC+1]);
            cplex.addEq(LHS2b, 1);
        }

        // 2c
        for (int i = nC; i <= nV + nC; i++) {
            IloNumExpr LHS2c = cplex.constant(0);
            for (int j = 1; j <= nV; j++) {
                if (j != i) {
                    LHS2c = cplex.sum(LHS2c, z_matrix[i][j]);
                }
            }
            // Including also the arc goes to the ending depot
            LHS2c = cplex.sum(LHS2c, z_matrix[i][nV+nC+1]);
            cplex.addLe(LHS2c, 1);
        }

        // 2d
        for (int k = 1; k <= nV + nC; k++) {
            IloNumExpr LHS2d = cplex.constant(0);
            for (int i = 0; i <= nV + nC; i++) {
                if (i != k) {
                    LHS2d = cplex.sum(LHS2d, z_matrix[i][k]);
                }
            }

            for (int j = 1; j <= nV + nC + 1; j++) {
                if (j != k) {
                    LHS2d = cplex.diff(LHS2d, z_matrix[k][j]);
                }
            }
            cplex.addEq(LHS2d, 0);
        }

        // 2.4
//        IloNumExpr LHS4 = cplex.constant(0);
//        for (int j = 1; j < nLocations-1; j++) {
//            LHS4 = cplex.sum(LHS4, z_matrix[0][j]);
//        }
//        cplex.addGe(LHS4, 0);

        // 2e + 2f
        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j < nLocations; j++) {
                IloNumExpr chargedOrNot = cplex.constant(0);
                for (int k = nV+1; k <= nV + nC; k++) {
                    chargedOrNot = cplex.sum(chargedOrNot, z_matrix[k][i]);
                }
                chargedOrNot = cplex.diff(1, chargedOrNot);
                
                IloNumExpr RHS_time = cplex.diff(cplex.sum(cplex.prod(t_vector[i], chargedOrNot), 
                                                           cplex.prod(t_matrix[i][j], z_matrix[i][j])),
                                                 cplex.prod(T, cplex.diff(1, z_matrix[i][j])));
                cplex.addGe(t_vector[j], RHS_time);

                IloNumExpr RHS_charge = cplex.diff(cplex.sum(q_vector[i], cplex.prod(q_matrix[i][j], z_matrix[i][j])),
                        cplex.prod(Q, cplex.diff(1, z_matrix[i][j])));
                cplex.addGe(q_vector[j], RHS_charge);
            }
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
                    while (to <= 21) {
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