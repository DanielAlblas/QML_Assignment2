import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

import java.util.ArrayList;

public class ConvexQuadraticProgram {
    private IloCplex cplex;
    private IloNumVar[] xi;
    private ArrayList<Double> zeta_vector;
    private ArrayList<Double> tau_vector;

    private int p;
    private double Q;
    private double T;

    public ConvexQuadraticProgram(ArrayList<Double> chargeUsed, double maxCharge, ArrayList<Double> timeElapsed, double maxTime) throws IloException {
        // Initialize the cplex solver
        cplex = new IloCplex();

        zeta_vector = chargeUsed;
        tau_vector = timeElapsed;
        p = zeta_vector.size() - 1;
        Q = maxCharge;
        T = maxTime;
        zeta_vector.add(0.0);
        tau_vector.add(0.0);

        xi = new IloNumVar[p + 2]; // 0 and p+1 are the depot
        for (int i = 1; i <= p; i++) {
            xi[i] = cplex.numVar(0, Double.POSITIVE_INFINITY);
        }
    }

    public boolean routeIsFeasible() throws IloException {
        // Create the objective function
        IloNumExpr obj = cplex.constant(0);
        for (int i = 1; i <= p; i++) {
            obj = cplex.sum(obj, cplex.prod(xi[i], xi[i]));
        }
        cplex.addMinimize(obj);

        // Constraint 1
        for (int j = 1; j <= p; j++) {
            IloNumExpr LHS1 = cplex.constant(0);
            for (int i = 1; i <= j; i++) {
                LHS1 = cplex.sum(LHS1, xi[i]);
            }
            double RHS1 = 0;
            for (int i = 0; i <= j - 1; i++) {
                RHS1 += zeta_vector.get(i);
            }
            cplex.addLe(LHS1, RHS1);
        }

        // Constraint 2
        for (int j = 1; j <= p; j++) {
            IloNumExpr LHS2 = cplex.constant(0);
            for (int i = 1; i <= j; i++) {
                LHS2 = cplex.sum(LHS2, xi[i]);
            }
            double RHS2 = -Q;
            for (int i = 0; i <= j; i++) {
                RHS2 += zeta_vector.get(i);
            }
            cplex.addGe(LHS2, RHS2);
        }

        cplex.setOut(null);
        cplex.solve();

        // Query the solution
        if (cplex.getStatus() == IloCplex.Status.Optimal) {
            double LHS = cplex.getObjValue() / 100;
            for (int i = 0; i <= p; i++) {
                LHS += tau_vector.get(i);
            }
            System.out.println("new route");
            if (LHS > T) {
                return false;
            } else {
                if (zeta_vector.size() > 2) {
                    for (int i = 1; i <= p; i++) {
                        double time_spent_at_station_i = cplex.getValue(xi[i]);
                        time_spent_at_station_i = time_spent_at_station_i*time_spent_at_station_i/100;
                        System.out.println("time spent at station " + i + " in above route: " + time_spent_at_station_i);
                    }
                }
                return true;
            }
        }
        return false;
    }
}
