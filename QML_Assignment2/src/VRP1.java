import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

import javax.sound.midi.Soundbank;

public class VRP1 {
    private int nLocations;
    private double[][] d_matrix;
    private double[][] c_matrix;
    private double[][] q_matrix;
    private double[][] t_matrix;
    private double Q;
    private double T;
    private IloCplex cplex;
    private IloNumVar[][][] z_matrix;
    private IloNumVar[] q_vector;
    private IloNumVar[] t_vector;
    private IloNumVar[][] time_matrix;
    private IloNumVar[][] charge_matrix;
    private IloNumVar[][] u_matrix;

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
        z_matrix = new IloNumVar[nLocations][nLocations][nLocations];
        q_vector = new IloNumVar[nLocations];
        t_vector = new IloNumVar[nLocations];

        time_matrix = new IloNumVar[nLocations-1][nLocations];
        charge_matrix = new IloNumVar[nLocations-1][nLocations];
        u_matrix = new IloNumVar[nLocations-1][nLocations];

        //TODO: might be able to use nLocations-1 in the loops instead of below line 47
        for (int k = 0; k < nLocations; k++) {
            for (int i = 0; i < nLocations; i++) {
                for (int j = 0; j < nLocations; j++) {
                    z_matrix[i][j][k] = cplex.boolVar("Arc (" + i + "," + j + ")");
                }
            }
        }
        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j < nLocations; j++) {
                c_matrix[i][j] = 1 + d_matrix[i][j];
                q_matrix[i][j] = 10 + Math.pow(d_matrix[i][j], 0.75);
                t_matrix[i][j] = 5 + Math.pow(d_matrix[i][j], 0.9);
            }
        }

        for (int i = 0; i < nLocations-1; i++) {
            for (int k = 0; k < nLocations; k++) {
                u_matrix[i][k] = cplex.intVar(1,nLocations-1);
                time_matrix[i][k] = cplex.numVar(0, T);
                charge_matrix[i][k] = cplex.numVar(0, Q);
            }
        }

        q_matrix[0][nLocations - 1] = 0;
        t_matrix[0][nLocations - 1] = 0;
        q_matrix[nLocations - 1][0] = 0;
        t_matrix[nLocations - 1][0] = 0;

        for (int i = 0; i < nLocations; i++) {
            q_vector[i] = cplex.numVar(0, 50);
            t_vector[i] = cplex.numVar(0, 50);
        }

        c_matrix[0][nLocations - 1] = 0;
        c_matrix[nLocations - 1][0] = 0;
    }

    public void solveModel() throws IloException {
        // Create the objective function
        IloNumExpr obj = cplex.constant(0);
        for (int k = 0; k < nLocations; k++) {
            for (int i = 0; i < nLocations; i++) {
                for (int j = 0; j < nLocations; j++) {
                    obj = cplex.sum(obj, cplex.prod(c_matrix[i][j], z_matrix[i][j][k]));
                }
            }
        }

        cplex.addMinimize(obj);

        // Add the restrictions
        // LHS_visit ensures that every location is visited once total for every route
        for (int i = 1; i < nLocations - 1; i++) {
            IloNumExpr LHS_visit = cplex.constant(0);
            for (int k = 0; k < nLocations; k++) {
                for (int j = 0; j < nLocations; j++) {
                    LHS_visit = cplex.sum(LHS_visit, z_matrix[i][j][k]);
                }
            }
            cplex.addEq(LHS_visit, 1);
        }

        // LHS_start_at_depot ensures all vehicles leave the depot
        for (int k = 0; k < nLocations; k++) {
            IloNumExpr LHS_start_at_depot = cplex.constant(0);
            for (int j = 0; j < nLocations; j++) {
                LHS_start_at_depot = cplex.sum(LHS_start_at_depot, z_matrix[0][j][k]);
            }
            cplex.addEq(LHS_start_at_depot, 1);
        }

        // LHS_same_route ensures that if a route visits a location, the route to the following location is travelled by the same vehicle
        for (int k = 0; k < nLocations; k++) {
            for (int l = 1; l < nLocations - 1; l++) {
                IloNumExpr LHS_same_route = cplex.constant(0);
                for (int i = 0; i < nLocations; i++) {
                    LHS_same_route = cplex.sum(LHS_same_route, z_matrix[i][l][k]);
                }
                for (int j = 0; j < nLocations; j++) {
                    LHS_same_route = cplex.sum(LHS_same_route, cplex.prod(-1, z_matrix[l][j][k]));
                }
                cplex.addEq(LHS_same_route, 0);
            }
        }

        // LHS_end_at_depot ensures all vehicles return to the depot
        for (int k = 0; k < nLocations; k++) {
            IloNumExpr LHS_end_at_depot = cplex.constant(0);
            for (int i = 0; i < nLocations; i++) {
                LHS_end_at_depot = cplex.sum(LHS_end_at_depot, z_matrix[i][nLocations - 1][k]);
            }
            cplex.addEq(LHS_end_at_depot, 1);
        }

        for (int k = 0; k < nLocations; k++) {
            for (int i = 0; i < nLocations; i++) {
                cplex.addEq(z_matrix[i][i][k], 0);
                cplex.addEq(z_matrix[nLocations-1][i][k], 0);
                cplex.addEq(z_matrix[i][0][k], 0);
            }
        }

        for (int k = 0; k < nLocations; k++) {
            for (int i = 0; i < nLocations-1; i++) {
                for (int j = 0; j < nLocations-1; j++) {
                    cplex.addGe(time_matrix[j][k], cplex.diff(cplex.sum(time_matrix[i][k], t_matrix[i+1][j+1]), cplex.prod(T, cplex.diff(1, z_matrix[i+1][j+1][k]))));
                    cplex.addGe(charge_matrix[j][k], cplex.diff(cplex.sum(charge_matrix[i][k], q_matrix[i+1][j+1]), cplex.prod(Q, cplex.diff(1, z_matrix[i+1][j+1][k]))));
                }
                cplex.addLe(cplex.sum(time_matrix[i][k], t_matrix[i][21]), T);
                cplex.addLe(cplex.sum(charge_matrix[i][k], q_matrix[i][21]), Q);
            }
        }



//        for (int k = 0; k < nLocations; k++) {
//            for (int i = 0; i < nLocations-1; i++) {
//                for (int j = 0; j < nLocations-1; j++) {
//                    cplex.addGe(u_matrix[j][k], cplex.diff(cplex.sum(u_matrix[i][k], 1), cplex.prod(nLocations-3, cplex.diff(1, z_matrix[i+1][j+1][k]))));
//                }
//            }
//        }
//
//        for (int k = 0; k < nLocations; k++) {
//            IloNumExpr LHS_time_maximum = cplex.constant(0);
//            for (int i = 0; i < nLocations; i++) {
//                for (int j = 0; j < nLocations; j++) {
//                    LHS_time_maximum = cplex.sum(LHS_time_maximum, cplex.prod(t_matrix[i][j], z_matrix[i][j][k]));
//                }
//            }
//            cplex.addLe(LHS_time_maximum, Q);
//        }
//
//        for (int k = 0; k < nLocations; k++) {
//            IloNumExpr LHS_charge_maximum = cplex.constant(0);
//            for (int i = 0; i < nLocations; i++) {
//                for (int j = 0; j < nLocations; j++) {
//                    LHS_charge_maximum = cplex.sum(LHS_charge_maximum, cplex.prod(q_matrix[i][j], z_matrix[i][j][k]));
//                }
//            }
//            cplex.addLe(LHS_charge_maximum, Q);
//        }

        //potential bigM calculation (wrong for now)
//        double bigM = Q - q_matrix[1][nLocations-1] - q_matrix[1][0];
//        for (int i = 0; i < nLocations; i++) {
//            for (int j = 0; j < nLocations; j++) {
//                if (i != j) {
//                    double tempBigM = 0;
//                    tempBigM = Q - q_matrix[i][nLocations-1] - q_matrix[i][j];
//                    bigM = Math.max(bigM, tempBigM);
//                }
//            }
//        }
//        System.out.println(bigM);
//
//
//        double bigM = Q;
//
//        for (int k = 0; k < nLocations; k++) {
//            cplex.addEq(charge_matrix[0][k], 0);
//            for (int i = 0; i < nLocations; i++) {
//                for (int j = 0; j < nLocations; j++) {
//                    cplex.addLe(cplex.diff(cplex.sum(charge_matrix[i][k], q_matrix[i][j]), cplex.prod(bigM, cplex.diff(1, z_matrix[i][j][k]))), charge_matrix[j][k]);
//                }
//                cplex.addLe(charge_matrix[i][k], Q - q_matrix[i][nLocations - 1]);
//            }
//        }
//
//        double bigM2 = T;
//
//        for (int k = 0; k < nLocations; k++) {
//            cplex.addEq(time_matrix[0][k], 0);
//            for (int i = 0; i < nLocations; i++) {
//                for (int j = 0; j < nLocations; j++) {
//                    cplex.addLe(cplex.diff(cplex.sum(time_matrix[i][k], t_matrix[i][j]), cplex.prod(bigM2, cplex.diff(1, z_matrix[i][j][k]))), time_matrix[j][k]);
//                }
//                cplex.addLe(time_matrix[i][k], T - t_matrix[i][nLocations - 1]);
//            }
//        }


        //summed
//        for (int i = 0; i < nLocations; i++) {
//            for (int j = 0; j < nLocations; j++) {
//                if (i != j) {
//                    IloNumExpr LHS_subtour_elimination_time = cplex.constant(0);
//                    IloNumExpr LHS_subtour_elimination_charge = cplex.constant(0);
//                    LHS_subtour_elimination_time = cplex.sum(LHS_subtour_elimination_time, t_vector[i]);
//                    LHS_subtour_elimination_charge = cplex.sum(LHS_subtour_elimination_charge, q_vector[i]);
//                    for (int k = 0; k < nLocations; k++) {
//                        LHS_subtour_elimination_time = cplex.sum(LHS_subtour_elimination_time, cplex.prod(T, z_matrix[i][j][k]));
//                        LHS_subtour_elimination_time = cplex.sum(LHS_subtour_elimination_time, cplex.prod(t_matrix[i][j], z_matrix[i][j][k]));
//
//                        LHS_subtour_elimination_charge = cplex.sum(LHS_subtour_elimination_charge, cplex.prod(Q, z_matrix[i][j][k]));
//                        LHS_subtour_elimination_charge = cplex.sum(LHS_subtour_elimination_charge, cplex.prod(q_matrix[i][j], z_matrix[i][j][k]));
//                    }
//                    cplex.addEq(LHS_subtour_elimination_time, cplex.sum(t_vector[j], T));
//                    cplex.addEq(LHS_subtour_elimination_charge, cplex.sum(q_vector[j], Q));
//                }
//            }
//        }
//
//        for (int j = 1; j < nLocations - 1; j++) {
//            IloNumExpr LHS_time_constraint = cplex.constant(0);
//            IloNumExpr LHS_charge_constraint = cplex.constant(0);
//            LHS_time_constraint = cplex.sum(LHS_time_constraint, t_vector[j]);
//            LHS_charge_constraint = cplex.sum(LHS_charge_constraint, q_vector[j]);
//            for (int k = 0; k < nLocations; k++) {
//                LHS_time_constraint = cplex.sum(LHS_time_constraint, cplex.prod(t_matrix[j][nLocations - 1], z_matrix[j][nLocations - 1][k]));
//                LHS_charge_constraint = cplex.sum(LHS_charge_constraint, cplex.prod(q_matrix[j][nLocations - 1], z_matrix[j][nLocations - 1][k]));
//            }
//            cplex.addLe(LHS_time_constraint, T);
//            cplex.addLe(LHS_charge_constraint, Q);
//        }

        //more constraints
//        for (int k = 0; k < nLocations; k++) {
//            for (int i = 0; i < nLocations; i++) {
//                for (int j = 0; j < nLocations; j++) {
//                    if (i != j) {
//                        IloNumExpr LHS_subtour_elimination_time = cplex.constant(0);
//                        LHS_subtour_elimination_time = cplex.sum(LHS_subtour_elimination_time, cplex.prod(T, z_matrix[i][j][k]));
//                        LHS_subtour_elimination_time = cplex.sum(LHS_subtour_elimination_time, cplex.prod(t_matrix[i][j], z_matrix[i][j][k]));
//                        LHS_subtour_elimination_time = cplex.sum(LHS_subtour_elimination_time, t_vector[i]);
//
//                        IloNumExpr LHS_subtour_elimination_charge = cplex.constant(0);
//                        LHS_subtour_elimination_charge = cplex.sum(LHS_subtour_elimination_charge, cplex.prod(Q, z_matrix[i][j][k]));
//                        LHS_subtour_elimination_charge = cplex.sum(LHS_subtour_elimination_charge, cplex.prod(q_matrix[i][j], z_matrix[i][j][k]));
//                        LHS_subtour_elimination_charge = cplex.sum(LHS_subtour_elimination_charge, q_vector[i]);
//
//                        cplex.addEq(LHS_subtour_elimination_time, cplex.sum(t_vector[j], T));
//                        cplex.addEq(LHS_subtour_elimination_charge, cplex.sum(q_vector[j], Q));
//                    }
//                }
//            }
//        }
//
//        for (int k = 0; k < nLocations; k++) {
//            for (int j = 1; j < nLocations; j++) {
//                cplex.addLe(cplex.sum(t_vector[j], cplex.prod(t_matrix[j][nLocations-1], z_matrix[j][nLocations-1][k])), T);
//                cplex.addLe(cplex.sum(q_vector[j], cplex.prod(q_matrix[j][nLocations-1], z_matrix[j][nLocations-1][k])), Q);
//            }
//        }

        // Solve the model
        cplex.setOut(null);
        cplex.solve();

        // Query the solution
        if (cplex.getStatus() == IloCplex.Status.Optimal) {
            System.out.println("Found optimal solution!");
            System.out.println("Objective = " + cplex.getObjValue());
            int route = 1;
            for (int k = 0; k < nLocations; k++) {
                int routeLocations = 0;
                for (int j = 0; j < nLocations; j++) {
                    for (int i = 0; i < nLocations; i++) {
                        routeLocations += cplex.getValue(z_matrix[i][j][k]);
                    }
                }
                if (routeLocations >= 1.5) {
                    System.out.println("Route: " + route);
                    for (int j = 0; j < nLocations; j++) {
                        for (int i = 0; i < nLocations; i++) {
                            if (cplex.getValue(z_matrix[i][j][k]) >= 0.5) {
                                System.out.println("Location " + i + ", " + j);
//                        System.out.println("Charge:" + q_vector[i] + ", time: " + t_vector[i]);
                            }
                        }
                    }
                    route++;
                }
            }
        } else {
            System.out.println("No optimal solution found");
        }

        // Close the model
        cplex.close();
    }
}
