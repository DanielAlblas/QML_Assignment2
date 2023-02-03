import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

import java.util.List;
import java.util.ArrayList;

public class ChargingVRP {
    private int nLocations;
    private int nV;
    private int nC;
    private ArrayList<IloNumExpr> LHS_cuts;
    private ArrayList<Integer> RHS_cuts;
    private double[][] d_matrix;
    private double[][] c_matrix;
    private double[][] q_matrix;
    private double[][] t_matrix;
    private double Q;
    private double T;
    private IloCplex cplex;
    private IloNumVar[][] z_matrix;
    private IloNumVar[] eta_vector;
    private IloNumVar[] psi_vector;

    public ChargingVRP(double[][] distances, double maxElectricity, double maxTime, int numLocations, int numChargeStations, ArrayList<IloNumExpr> LHS_cut, ArrayList<Integer> RHS_cut) throws IloException {
        // Initialize the cplex solver
        cplex = new IloCplex();

        nLocations = distances.length;
        d_matrix = distances;
        Q = maxElectricity;
        T = maxTime;
        nV = numLocations;
        nC = numChargeStations;
        LHS_cut = LHS_cut;
        RHS_cut = RHS_cut;

        c_matrix = new double[nLocations][nLocations];
        q_matrix = new double[nLocations][nLocations];
        t_matrix = new double[nLocations][nLocations];

        // Creating the decision variables: matrix x and vector y
        z_matrix = new IloNumVar[nLocations][nLocations];
        eta_vector = new IloNumVar[nLocations];
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
            eta_vector[i] = cplex.numVar(0, Q);
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

        // Add cuts
        if (LHS_cuts != null) {
            for (int i = 0; i < LHS_cuts.size(); i++) {
                cplex.addLe(LHS_cuts.get(i), RHS_cuts.get(i));
            }
        }

        // Disallows travel to and form the same location
        for (int i = 0; i < nLocations; i++) {
            cplex.addEq(z_matrix[i][i], 0);
        }
        cplex.addEq(z_matrix[0][nLocations - 1], 0);

        // Visit constraints for buyers (2b)
        for (int i = 1; i <= nV; i++) {
            IloNumExpr LHS2b = cplex.constant(0);
            for (int j = 1; j < nLocations; j++) {
                if (j != i) {
                    LHS2b = cplex.sum(LHS2b, z_matrix[i][j]);
                }
            }
            cplex.addEq(LHS2b, 1);
        }

        // Visit constraints for charging stations (2c)
        for (int i = nV + 1; i <= nV + nC; i++) {
            IloNumExpr LHS2c = cplex.constant(0);
            for (int j = 1; j < nLocations; j++) {
                if (j != i) {
                    LHS2c = cplex.sum(LHS2c, z_matrix[i][j]);
                }
            }
            cplex.addLe(LHS2c, 1);
        }

        // Constraints to ensure same route (2d)
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

        // Time constraints to disallow subtours (2e)
        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j < nLocations; j++) {
                if (i != j) {
                    IloNumExpr RHS_time = cplex.diff(cplex.sum(psi_vector[i], cplex.prod(t_matrix[i][j], z_matrix[i][j])),
                            cplex.prod(T, cplex.diff(1, z_matrix[i][j])));
                    cplex.addGe(psi_vector[j], RHS_time);
                }
            }
        }

        // Constraints to limit time (2f)
        for (int j = 1; j < nLocations - 1; j++) {
            cplex.addLe(t_matrix[0][j], psi_vector[j]);
            cplex.addLe(psi_vector[j], T - t_matrix[j][nLocations - 1]);
        }

        // Charge constraints to disallow subtours (2g)
        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j <= nV; j++) {
                if (i != j) {
                    IloNumExpr RHS_charge = cplex.sum(cplex.diff(eta_vector[i], cplex.prod(q_matrix[i][j], z_matrix[i][j])),
                            cplex.prod(Q, cplex.diff(1, z_matrix[i][j])));
                    cplex.addLe(eta_vector[j], RHS_charge);
                }
            }
        }

        // Constraints to limit charge (2h)
        for (int j = 1; j <= nV; j++) {
            for (int k = nV + 1; k < nLocations; k++) {
                cplex.addGe(eta_vector[j], cplex.prod(q_matrix[j][k], z_matrix[j][k]));
            }
        }

        // Constraints to recharge and leave depot with full charge (2i)
        for (int j = nV + 1; j <= nV + nC; j++) {
            cplex.addEq(eta_vector[j], Q);
        }
        cplex.addEq(eta_vector[0], Q);

        cplex.setOut(null);
        cplex.solve();

        // Query the solution
        if (cplex.getStatus() == IloCplex.Status.Optimal) {
            int cuts = 0;
            ArrayList<IloNumExpr> LHS_cuts_new = new ArrayList<>();
            ArrayList<Integer> RHS_cuts_new = new ArrayList<>();
            if (LHS_cuts != null) {
                LHS_cuts_new.addAll(LHS_cuts);
                RHS_cuts_new.addAll(RHS_cuts);
            }
            for (int i = 0; i < nLocations; i++) {
                if (cplex.getValue(z_matrix[0][i]) >= 0.5) {
                    int from = 0;
                    ArrayList<Integer> route = new ArrayList<>();
                    int to = i;
                    double charge = 0;
                    double time = 0;
                    ArrayList<Double> zeta_vector = new ArrayList<>();
                    ArrayList<Double> tau_vector = new ArrayList<>();
                    while (to < nLocations) {
                        if (cplex.getValue(z_matrix[from][to]) >= 0.5) {
                            route.add(from);
                            charge += q_matrix[from][to];
                            time += t_matrix[from][to];
                            if (to >= 21 && to <= 26) {
                                zeta_vector.add(charge);
                                charge = 0;
                                tau_vector.add(time);
                                time = 0;
                            }
                            //TODO: might need to add 27
                            from = to;
                            to = 0;
                        } else {
                            to++;
                        }
                    }
                    ConvexQuadraticProgram step2 = new ConvexQuadraticProgram(zeta_vector, Q, tau_vector, T);
                    if (!step2.routeIsFeasible()) {
                        cuts++;
                        IloNumExpr LHS_new_cut = cplex.constant(0);
                        for (int j = 0; j < route.size() - 1; j++) {
                            LHS_new_cut = cplex.sum(LHS_new_cut, z_matrix[route.get(j)][route.get(j + 1)]);
                        }
                        cuts++;
                        IloNumExpr LHS_new_cut_reverse = cplex.constant(0);
                        for (int j = route.size() - 2; j >= 0; j--) {
                            LHS_new_cut_reverse = cplex.sum(LHS_new_cut, z_matrix[route.get(j + 1)][route.get(j)]);
                        }
                        LHS_cuts_new.add(LHS_new_cut);
                        RHS_cuts_new.add(route.size() - 1);
                        LHS_cuts_new.add(LHS_new_cut_reverse);
                        RHS_cuts_new.add(route.size() - 1);
                    }
                }
            }
            if (cuts > 0) {
                ChargingVRP model = new ChargingVRP(d_matrix, Q, T, nV, nC, LHS_cuts_new, RHS_cuts_new);
                model.solveModel();
            } else {
                System.out.println("Found optimal solution!");
                System.out.println("Objective = " + cplex.getObjValue());

                int route = 0;
                for (int i = 0; i < nLocations; i++) {
                    if (cplex.getValue(z_matrix[0][i]) >= 0.5) {
                        route++;
                        System.out.print("Route " + route + ": 0");
                        int from = 0;
                        int to = i;
                        int numStationsVisited = 0;
                        List<Double> totalChargedAtStations = new ArrayList<>();
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
                                if (to >= 21 && to <= 26) {
                                    totalChargedAtStations.add(totalCharge);
                                }
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
                        System.out.print("Total used charge when the vehicle arrives at the charging stations: ");
                        if (!totalChargedAtStations.isEmpty()) {
                            for (Double c : totalChargedAtStations) {
                                System.out.print(c + "  ");
                            }
                        }
                        System.out.println();
                        System.out.println();
                    }
                }
            }
        } else {
            System.out.println("No optimal solution found");
        }

        // Close the model
        cplex.close();
    }
}
