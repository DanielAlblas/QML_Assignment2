import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

public class ConvexQuadraticProgram {
    private IloCplex cplex;
    private IloNumVar[] epsilon;
    private double[] zeta;
    private int p;
    private double Q;

    public ConvexQuadraticProgram(double[] chargeUsed, int numStations, double maxCharge) throws IloException {
        // Initialize the cplex solver
        cplex = new IloCplex();

        p = numStations;
        zeta = chargeUsed; // NOTE THAT zeta[0] AND zeta[p+1] NEED TO BE INITIALIZED EITHER HERE OR IN THE MAIN CLASS
        Q = maxCharge;

        epsilon = new IloNumVar[p+2]; // 0 and p+1 are the depot
        for (int i = 0; i < p+2; i++) {
            epsilon[i] = cplex.numVar(0, Double.POSITIVE_INFINITY);
        }
    }

    public void solveModel() throws IloException {
        // Create the objective function
        IloNumExpr obj = cplex.constant(0);
        for (int i = 1; i <= p; i++) {
            obj = cplex.sum(obj, cplex.prod(epsilon[i], epsilon[i]));
        }
        cplex.addMinimize(obj);

        // Constraint 1
        for (int j = 1; j <= p; j++) {
            IloNumExpr LHS1 = cplex.constant(0);
            for (int i = 1; i <= j; i++) {
                LHS1 = cplex.sum(LHS1, epsilon[i]);
            }
            double RHS1 = 0;
            for (int i = 0; i <= j-1 ; i++) {
                RHS1 += zeta[i];
            }
            cplex.addLe(LHS1, RHS1);
        }

        // Constraint 2
        for (int j = 1; j <= p; j++) {
            IloNumExpr LHS2 = cplex.constant(0);
            for (int i = 1; i <= j; i++) {
                LHS2 = cplex.sum(LHS2, epsilon[i]);
            }
            double RHS2 = -Q;
            for (int i = 0; i <= j ; i++) {
                RHS2 += zeta[i];
            }
            cplex.addLe(LHS2, RHS2);
        }

        // MIGHT NEED TO ADD CONSTRAINTS FOR epsilon[0] AND epsilon[p+1]
    }
}
