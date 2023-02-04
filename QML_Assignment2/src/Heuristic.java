
import java.util.*;

public class Heuristic {
    private int nLocations;
    private int nV;
    private int nC;
    private double[][] d_matrix;
    private double[][] c_matrix;
    private double[][] q_matrix;
    private double[][] t_matrix;
    private double Q;
    private double T;
    private ArrayList<ArrayList<Integer>> toursList;
    private ArrayList<ArrayList<Integer>> feasibleToursList;
    private ArrayList<ArrayList<Integer>> infeasibleToursList;
    private Map<Integer, ArrayList<Integer>> adjacentVertexToRoute;
    private Map<int[], Double> savingsPairList;
    private List<Map.Entry<int[], Double>> pairs;
    private List<Integer> chargingStations; //currently available to use
    private final ArrayList<ArrayList<Integer>> result = new ArrayList<>();

    public Heuristic(double[][] distances, double maxElectricity, double maxTime, int numLocations, int numChargeStations) {
        nLocations = distances.length; // = 28
        d_matrix = distances;
        Q = maxElectricity;
        T = maxTime;
        nV = numLocations;
        nC = numChargeStations;

        c_matrix = new double[nLocations][nLocations];
        q_matrix = new double[nLocations][nLocations];
        t_matrix = new double[nLocations][nLocations];
        for (int i = 0; i < nLocations; i++) {
            for (int j = 0; j < nLocations; j++) {
                c_matrix[i][j] = 1 + d_matrix[i][j];
                q_matrix[i][j] = 10 + Math.pow(d_matrix[i][j], 0.75);
                t_matrix[i][j] = 5 + Math.pow(d_matrix[i][j], 0.9);
            }
        }

        toursList = new ArrayList<>();
        for (int i = 1; i <= nV; i++) {
            toursList.add(new ArrayList<>(Arrays.asList(0, i, nLocations-1)));
        }

        feasibleToursList = new ArrayList<>();
        infeasibleToursList = new ArrayList<>();
        for (ArrayList<Integer> tour : toursList) {
            if (isFeasible(tour)) {
                feasibleToursList.add(tour);
            } else {
                infeasibleToursList.add(tour);
            }
        }

        adjacentVertexToRoute = new LinkedHashMap<>();
        savingsPairList = new LinkedHashMap<>();
        //call step 4
        step4();

        double totalCost = 0;
        System.out.printf("Solution:");
        for (ArrayList<Integer> tour : feasibleToursList) {
            System.out.print("Route: ");
            for (Integer location : tour) {
                System.out.printf(location + " ");
            }
            System.out.println();
            double cost = computeCost(tour);
            totalCost += cost;
            System.out.println("Cost: " + cost);
            System.out.println();
        }
        System.out.println(totalCost);
    }

    private void step4() {
        // Store all vertices that are adjacent to the depot in a tour
        for (ArrayList<Integer> tour : feasibleToursList) {
            adjacentVertexToRoute.put(tour.get(1), tour);
        }

        List<Integer> adjVertices = new ArrayList<>(adjacentVertexToRoute.keySet());
        Map<int[], Double> savingsPairListUnsorted = new LinkedHashMap<>();
        for (int i = 0; i < adjVertices.size()-1; i++) {
            for (int j = i+1; j < adjVertices.size()-1; j++) {
                if (i != j) {
                    int[] pair = new int[] {i, j};
                    double saving = c_matrix[0][i] + c_matrix[0][j] - c_matrix[i][j];
                    savingsPairListUnsorted.put(pair, saving);
                }
            }
        }

        pairs = new LinkedList<>(savingsPairListUnsorted.entrySet());
        Collections.sort(pairs,
                (map1, map2) -> map2.getValue().compareTo(map1.getValue())
        );


        for (Map.Entry<int[], Double> map : pairs) {
            savingsPairList.put(map.getKey(), map.getValue());
        }

        List<Integer> chargingStations = new ArrayList<>();
        for (int i = nV + 1; i <= nC+ nV; i++) {
            chargingStations.add(i);
        }

        step5();
    }

    private void step5() {
        int numFeasRoutes = feasibleToursList.size();
        while (!savingsPairList.isEmpty()) {
            int[] pair = pairs.get(0).getKey();
//            Double saving = savingsPairList.get(pair);
            savingsPairList.remove(pair);

            ArrayList<Integer> tour_i = adjacentVertexToRoute.get(pair[0]);
            ArrayList<Integer> tour_j = adjacentVertexToRoute.get(pair[1]);

            ArrayList<Integer> mergedTour = getMergedTour(tour_i, tour_j);
            ArrayList<Integer> mergedTourReverse = getMergedTour(tour_j, tour_i);

            if (isFeasible(mergedTour)) {
                feasibleToursList.add(mergedTour);
            } else if (isFeasible(mergedTourReverse)) {
                feasibleToursList.add(mergedTourReverse);
            } else if (isTimeFeasible(mergedTour) && isTimeFeasible(mergedTourReverse)) {
                ArrayList<Integer> removedCombinationIJ = getRedundantStations(pair, tour_i, tour_j, mergedTour);
                ArrayList<Integer> removedCombinationJI = getRedundantStations(new int[] {pair[1], pair[0]}, tour_j, tour_i, mergedTourReverse);

                if (removedCombinationIJ.size() > removedCombinationJI.size()) {
                    mergedTour.removeAll(removedCombinationIJ);
                    feasibleToursList.add(mergedTour);
                } else if (removedCombinationIJ.size() < removedCombinationJI.size()) {
                    mergedTourReverse.removeAll(removedCombinationIJ);
                    feasibleToursList.add(mergedTourReverse);
                } else {
                    mergedTour.removeAll(removedCombinationIJ);
                    mergedTourReverse.removeAll(removedCombinationJI);
                    double costIJ = computeCost(mergedTour);
                    double costJI = computeCost(mergedTourReverse);
                    if (costIJ >= costJI) {
                        feasibleToursList.add(mergedTourReverse);
                    } else {
                        feasibleToursList.add(mergedTour);
                    }
                }
            } else if (isTimeFeasible(mergedTour)) {
                ArrayList<Integer> removedCombinationIJ = getRedundantStations(pair, tour_i, tour_j, mergedTour);
                mergedTour.removeAll(removedCombinationIJ);
                feasibleToursList.add(mergedTour);
            } else if (isTimeFeasible(mergedTourReverse)) {
                ArrayList<Integer> removedCombinationJI = getRedundantStations(new int[] {pair[1], pair[0]}, tour_j, tour_i, mergedTourReverse);
                mergedTourReverse.removeAll(removedCombinationJI);
                feasibleToursList.add(mergedTourReverse);
            }
        }
        if (feasibleToursList.size() != numFeasRoutes) {
            step4();
        }
    }

    private ArrayList<Integer> getMergedTour(ArrayList<Integer> tour_i, ArrayList<Integer> tour_j) {
        ArrayList<Integer> mergedTour = new ArrayList<>();
        mergedTour.add(0);
        for (int i = 1; i < tour_i.size() - 1; i++) {
            mergedTour.add(tour_i.get(i));
        }
        for (int j = 1; j < tour_j.size() - 1; j++) {
            mergedTour.add(tour_j.get(j));
        }
        mergedTour.add(nLocations-1);
        return mergedTour;
    }

    private ArrayList<Integer> getRedundantStations(int[] pair, ArrayList<Integer> tour_i, ArrayList<Integer> tour_j, ArrayList<Integer> mergedTour) {
        ArrayList<Integer> bestComb = new ArrayList<>();
        tour_i.remove(tour_i.size() - 1);
        tour_j.remove(0);

        double minInsertionCost = Double.POSITIVE_INFINITY;
        int bestStation = 0;
        ArrayList<Integer> newMergedTour = new ArrayList<>();

        for (Integer f : chargingStations) {
            mergedTour = new ArrayList<>(tour_i);
            mergedTour.add(f);
            mergedTour.addAll(tour_j);

            double insertionCost = c_matrix[pair[0]][f] + c_matrix[f][pair[1]] - c_matrix[pair[0]][0] - c_matrix[pair[1]][0];
            if (insertionCost < minInsertionCost && isFeasible(mergedTour)) {
                bestStation = f;
                minInsertionCost = insertionCost;
                newMergedTour = new ArrayList<>(mergedTour);
            }
        }

        if (bestStation != 0) {
            // check for redundancy
            ArrayList<Integer> listStations = new ArrayList<>();
            for (Integer location : newMergedTour) {
                if (location >= 21 && location <= 26) {
                    listStations.add(location);
                }
            }

            if (listStations.size() > 1) {
                findAllCombinations(listStations);
            }

            List<ArrayList<Integer>> feasibleRemovals = new ArrayList<>();
            ArrayList<Integer> checkFeasibility = new ArrayList<>();
            for (ArrayList<Integer> comb : result) {
                checkFeasibility = new ArrayList<>(newMergedTour);
                checkFeasibility.removeAll(comb);
                if (isFeasible(checkFeasibility)) {
                    feasibleRemovals.add(comb);
                }
            }

            int maxCombSize = 0;
            for (ArrayList<Integer> comb : feasibleRemovals) {
                maxCombSize = Math.max(maxCombSize, comb.size());
            }

            int finalMaxCombSize = maxCombSize;
            feasibleRemovals.removeIf(comb -> comb.size() < finalMaxCombSize);

            double minSavingCost = Double.POSITIVE_INFINITY;
            ArrayList<Integer> temp = new ArrayList<>();
            for (ArrayList<Integer> comb : feasibleRemovals) {
                temp = new ArrayList<>(newMergedTour);
                temp.removeAll(comb);
                double cost = 0;
                for (int i = 0; i < temp.size()-1; i++) {
                    cost += c_matrix[i][i+1];
                }
                if (cost < minSavingCost) {
                    minSavingCost = cost;
                    bestComb = comb;
                }
            }
        }
        return bestComb;
    }

    private double computeCost(ArrayList<Integer> tour) {
        double cost = 0;
        for (int i = 0; i < tour.size() - 1; i++) {
            cost += c_matrix[tour.get(i)][tour.get(i+1)];
        }
        return cost;
    }

    // TODO: Consider the time spent in the charging stations
    private boolean isFeasible(ArrayList<Integer> tour) {
        double charge = 0;
        for (int i = 0; i < tour.size()-1; i++) {
            charge += q_matrix[tour.get(i)][tour.get(i+1)];
        }

        if (charge <= Q && isTimeFeasible(tour)) {
            return true;
        } else {
            return false;
        }
    }

    private boolean isTimeFeasible(ArrayList<Integer> tour) {
        double currentChargeLevel = 50;
        double time = 0;
        for (int i = 0; i < tour.size()-1; i++) {
            double xi = 0;
            if (tour.get(i) >= nV + 1 && tour.get(i) <= nV+nC) {
                xi = -currentChargeLevel;
                for (int j = i+1; j < tour.size()-1; j++) {
                    xi += q_matrix[tour.get(i)][tour.get(j)];

                    if (tour.get(j) >= nV + 1 && tour.get(j) <= nV+nC+1) {
                        break;
                    }
                }
            }
            time += t_matrix[tour.get(i)][tour.get(i+1)];
            time += xi * xi / 100;
            //TODO: might be wrong spot
            currentChargeLevel -= q_matrix[tour.get(i)][tour.get(i+1)];
        }

        if (time <= T) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Compute all combinations for a set L
     *
     * @param elements The set L for which all combinations are computed
     */
    public void findAllCombinations(ArrayList<Integer> elements) {
        result.clear();
        result.add(new ArrayList<>());
        List<ArrayList<Integer>> functionResult = new ArrayList<>();
        findCombinations(elements, 0, new ArrayList<>(), functionResult);
        result.addAll(functionResult);
    }

    private void findCombinations(List<Integer> elements, int startingIndex, List<Integer> currentPartition, List<ArrayList<Integer>> processResult) {
        for (int i = startingIndex; i < elements.size(); i++) {
            ArrayList<Integer> newPartition = new ArrayList<>(currentPartition);
            newPartition.add(elements.get(i));
            processResult.add(newPartition);
            findCombinations(elements, i + 1, newPartition, processResult);
        }
    }
}
