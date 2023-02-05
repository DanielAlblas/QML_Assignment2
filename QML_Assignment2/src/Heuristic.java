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
    private Map<Integer, ArrayList<Integer>> adjacentVertexToRoute;
    private Map<int[], Double> savingsPairList;
    private List<int[]> pairList;
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
            toursList.add(new ArrayList<>(Arrays.asList(0, i, nLocations - 1)));
        }

        chargingStations = new ArrayList<>();
        for (int i = nV + 1; i <= nC + nV; i++) {
            chargingStations.add(i);
        }

        // Call step 2
        step2();

        double totalCost = 0;
        System.out.println("Solution:");
        for (ArrayList<Integer> tour : toursList) {
            System.out.print("Route = ");
            for (Integer location : tour) {
                System.out.print(location + " ");
            }
            System.out.println();
            double cost = computeCost(tour);
            totalCost += cost;
            System.out.println("Cost = " + cost);
            System.out.println();
        }
        System.out.println("Objective = " + totalCost);
    }

    private void step2() {
        // Store all vertices that are adjacent to the leaving depot in a tour
        adjacentVertexToRoute = new LinkedHashMap<>();
        for (ArrayList<Integer> tour : toursList) {
            adjacentVertexToRoute.put(tour.get(1), tour);
        }

        // Create an unsorted SPL
        List<Integer> adjVertices = new ArrayList<>(adjacentVertexToRoute.keySet());
        Map<int[], Double> savingsPairListUnsorted = new LinkedHashMap<>();
        for (int i = 0; i < adjVertices.size(); i++) {
            for (int j = i + 1; j < adjVertices.size(); j++) {
                int location_i = adjVertices.get(i);
                int location_j = adjVertices.get(j);
                if (location_i != location_j && isInTour(adjacentVertexToRoute.get(location_i), location_j) == -1
                        && isInTour(adjacentVertexToRoute.get(location_j), location_i) == -1) {
                    int[] pair = new int[]{adjVertices.get(i), adjVertices.get(j)};
                    int sizeTourI = adjacentVertexToRoute.get(location_i).size();
                    int lastLocationInI = adjacentVertexToRoute.get(location_i).get(sizeTourI - 2);
                    double saving = c_matrix[lastLocationInI][nLocations-1] + c_matrix[0][location_j] - c_matrix[lastLocationInI][location_j];
                    if (saving > 0) {
                        savingsPairListUnsorted.put(pair, saving);
                    }

                    pair = new int[]{adjVertices.get(j), adjVertices.get(i)};
                    int sizeTourJ = adjacentVertexToRoute.get(location_j).size();
                    int lastLocationInJ = adjacentVertexToRoute.get(location_j).get(sizeTourJ - 2);
                    saving = c_matrix[lastLocationInJ][nLocations-1] + c_matrix[0][location_i] - c_matrix[lastLocationInJ][location_i];
                    if (saving > 0) {
                        savingsPairListUnsorted.put(pair, saving);
                    }
                }
            }
        }

        // Sort the SPL
        pairs = new LinkedList<>(savingsPairListUnsorted.entrySet());
        Collections.sort(pairs,
                (map1, map2) -> map2.getValue().compareTo(map1.getValue())
        );
        savingsPairList = new LinkedHashMap<>();
        pairList = new ArrayList<>();
        for (Map.Entry<int[], Double> map : pairs) {
            pairList.add(map.getKey());
            savingsPairList.put(map.getKey(), map.getValue());
        }

        // Call step 3
        step3();
    }

    private void step3() {
        int toursBeforeMerge = toursList.size();
        while (!savingsPairList.isEmpty()) {
            int n = toursList.size();
            int[] pair = pairList.get(0);

            savingsPairList.remove(pair);
            pairList.remove(pair);

            ArrayList<Integer> tour_i = new ArrayList<>();
            ArrayList<Integer> tour_j = new ArrayList<>();

            for (ArrayList<Integer> tour : toursList) {
                if (tour.get(1) == pair[0]) {
                    tour_i = tour;
                }
                if (tour.get(1) == pair[1]) {
                    tour_j = tour;
                }
            }

            ArrayList<Integer> mergedTour = getMergedTour(tour_i, tour_j);

            // If feasible we can replace the tour immediately
            if (isFeasible(mergedTour)) {
                toursList.remove(tour_i); //remove also the tours that are merged
                toursList.remove(tour_j); //remove also the tours that are merged
                toursList.add(mergedTour);
            }
            // Else we check if it is time feasible, and try to insert a station
            else if (isTimeFeasible(mergedTour)) {
                mergedTour = insertedStationTour(pair, tour_i, tour_j, mergedTour);
                if (mergedTour == null) {
                    break;
                }
                ArrayList<Integer> stationsInRoute = stationsInRoute(mergedTour);
                if (stationsInRoute.size() > 1) {
                    ArrayList<Integer> removedCombinationIJ = getRedundantStations(mergedTour, stationsInRoute);
                    mergedTour.removeAll(removedCombinationIJ);
                    chargingStations.addAll(removedCombinationIJ); // Added
                }
                toursList.remove(tour_i); //remove also the tours that are merged
                toursList.remove(tour_j); //remove also the tours that are merged
                toursList.add(mergedTour);
            }
            // We remove all pairs that include vertices used to replace the tours
            if (n != toursList.size()) {
                List<int[]> listToBeRemoved = new ArrayList<>();
                for (int[] p : pairList) {
                    if (p[0] == pair[0] || p[1] == pair[0] || p[0] == pair[1] || p[1] == pair[1]) {
                        listToBeRemoved.add(p);

                    }
                }
                for (int[] p : listToBeRemoved) {
                    savingsPairList.remove(p);
                    pairList.remove(p);
                }
            }
        }
        // If we replaced any tours we call step 2
        if (toursList.size() != toursBeforeMerge) {
            step2();
        }

    }

    private ArrayList<Integer> stationsInRoute(ArrayList<Integer> mergedTour) {
        ArrayList<Integer> listStations = new ArrayList<>();
        for (Integer location : mergedTour) {
            if (location >= 21 && location <= 26) {
                listStations.add(location);
            }
        }
        return listStations;
    }

    private ArrayList<Integer> insertedStationTour(int[] pair, ArrayList<Integer> tour1, ArrayList<Integer> tour2, ArrayList<Integer> mergedTour) {
        ArrayList<Integer> tour_i = new ArrayList<>(tour1);
        ArrayList<Integer> tour_j = new ArrayList<>(tour2);
        tour_i.remove(tour_i.size() - 1);
        tour_j.remove(0);

        double minInsertionCost = Double.POSITIVE_INFINITY;
        int bestStation = 0;
        ArrayList<Integer> newMergedTour = new ArrayList<>();

        for (Integer f : chargingStations) {
            mergedTour = new ArrayList<>(tour_i);
            mergedTour.add(f);
            mergedTour.addAll(tour_j);

            int location_j = pair[1];
            int lastLocationInI = tour_i.get(tour_i.size() - 1);
            double insertionCost = c_matrix[lastLocationInI][f] + c_matrix[f][location_j] - c_matrix[lastLocationInI][nLocations-1] - c_matrix[0][location_j];
            if (insertionCost < minInsertionCost && isFeasible(mergedTour)) {
                bestStation = f;
                minInsertionCost = insertionCost;
                newMergedTour = new ArrayList<>(mergedTour);
            }
        }
        // If a station can be added, add the best one
        if (bestStation != 0) {
            int finalBestStation = bestStation;
            chargingStations.removeIf(station -> station == finalBestStation);
            return newMergedTour;
        }
        // Else, return null (will be used in step 5 to determine if a station could be added)
        else {
            return null;
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
        mergedTour.add(nLocations - 1);
        return mergedTour;
    }

    private ArrayList<Integer> getRedundantStations(ArrayList<Integer> mergedTour, ArrayList<Integer> stationsInRoute) {
        List<ArrayList<Integer>> feasibleRemovals = new ArrayList<>();
        ArrayList<Integer> checkFeasibility;
        findAllCombinations(stationsInRoute);
        for (ArrayList<Integer> comb : result) {
            checkFeasibility = new ArrayList<>(mergedTour);
            checkFeasibility.removeAll(comb);
            if (isFeasible(checkFeasibility)) {
                feasibleRemovals.add(comb);
            }
        }

        int maxCombSize = 0;
        for (ArrayList<Integer> comb : feasibleRemovals) {
            maxCombSize = Math.max(maxCombSize, comb.size());
        }

        // Remove all feasible combination options that are not of the maximum size
        int finalMaxCombSize = maxCombSize;
        feasibleRemovals.removeIf(comb -> comb.size() < finalMaxCombSize);

        // Find the combination option with minimum cost
        double minSavingCost = Double.POSITIVE_INFINITY;
        ArrayList<Integer> bestComb = new ArrayList<>();
        ArrayList<Integer> temp;
        for (ArrayList<Integer> comb : feasibleRemovals) {
            temp = new ArrayList<>(mergedTour);
            temp.removeAll(comb);
            double cost = 0;
            for (int i = 0; i < temp.size() - 1; i++) {
                cost += c_matrix[i][i + 1];
            }
            if (cost < minSavingCost) {
                minSavingCost = cost;
                bestComb = comb;
            }
        }
        return bestComb;
    }

    // Check if a location is part of the tour + return its position in the tour (return -1 if it is not part of the tour)
    private int isInTour(ArrayList<Integer> tour, int location) {
        for (int i = 0; i < tour.size(); i++) {
            if (tour.get(i) == location) {
                return i;
            }
        }
        return -1;
    }

    private double computeCost(ArrayList<Integer> tour) {
        double cost = 0;
        for (int i = 0; i < tour.size() - 1; i++) {
            cost += c_matrix[tour.get(i)][tour.get(i + 1)];
        }
        return cost;
    }

    private boolean isFeasible(ArrayList<Integer> tour) {
        if (isChargeFeasible(tour) && isTimeFeasible(tour)) {
            return true;
        } else {
            return false;
        }
    }

    private boolean isChargeFeasible(ArrayList<Integer> tour) {
        double currentChargeLevel = 50;
        for (int i = 0; i < tour.size() - 1; i++) {
            double xi = 0;
            if (tour.get(i) >= nV + 1 && tour.get(i) <= nV + nC) {
                xi = -currentChargeLevel;
                for (int j = i; j < tour.size() - 1; j++) {
                    xi += q_matrix[tour.get(j)][tour.get(j + 1)];

                    if (tour.get(j + 1) >= nV + 1 && tour.get(j + 1) <= nV + nC + 1) {
                        break;
                    }
                }
            }
            xi = Math.min(Q, xi);
            currentChargeLevel += Math.max(0, xi);
            currentChargeLevel -= q_matrix[tour.get(i)][tour.get(i + 1)];
        }
        if (currentChargeLevel >= 0) {
            return true;
        } else {
            return false;
        }
    }

    private boolean isTimeFeasible(ArrayList<Integer> tour) {
        double currentChargeLevel = 50;
        double time = 0;
        for (int i = 0; i < tour.size() - 1; i++) {
            double xi = 0;
            if (tour.get(i) >= nV + 1 && tour.get(i) <= nV + nC) {
                xi = -currentChargeLevel;
                for (int j = i; j < tour.size() - 1; j++) {
                    xi += q_matrix[tour.get(j)][tour.get(j + 1)];

                    if (tour.get(j + 1) >= nV + 1 && tour.get(j + 1) <= nV + nC + 1) {
                        break;
                    }
                }
            }
            xi = Math.min(Q, xi);
            xi = Math.max(0, xi);
            time += t_matrix[tour.get(i)][tour.get(i + 1)];
            time += xi * xi / 100;
            currentChargeLevel -= q_matrix[tour.get(i)][tour.get(i + 1)];
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
