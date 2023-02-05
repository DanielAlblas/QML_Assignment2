
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

        feasibleToursList = new ArrayList<>();
        infeasibleToursList = new ArrayList<>();
        for (ArrayList<Integer> tour : toursList) {
            if (isFeasible(tour)) {
                feasibleToursList.add(tour);
            } else {
                infeasibleToursList.add(tour);
            }
        }

        chargingStations = new ArrayList<>();
        for (int i = nV + 1; i <= nC + nV; i++) {
            chargingStations.add(i);
        }

//        adjacentVertexToRoute = new LinkedHashMap<>();
//        savingsPairList = new LinkedHashMap<>();
        //call step 4
        step4();

        double totalCost = 0;
        System.out.println("Solution:");
        for (ArrayList<Integer> tour : feasibleToursList) {
            System.out.print("Route: ");
            for (Integer location : tour) {
                System.out.print(location + " ");
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
        adjacentVertexToRoute = new LinkedHashMap<>();
        for (ArrayList<Integer> tour : feasibleToursList) {
            adjacentVertexToRoute.put(tour.get(1), tour);
        }

        List<Integer> adjVertices = new ArrayList<>(adjacentVertexToRoute.keySet());
        Map<int[], Double> savingsPairListUnsorted = new LinkedHashMap<>();
        for (int i = 0; i < adjVertices.size() - 1; i++) {
            for (int j = i + 1; j < adjVertices.size() - 1; j++) {
                int location_i = adjVertices.get(i);
                int location_j = adjVertices.get(j);
                if (location_i != location_j && isInTour(adjacentVertexToRoute.get(location_i), location_j) == -1
                        && isInTour(adjacentVertexToRoute.get(location_j), location_i) == -1) {
                    int[] pair = new int[]{adjVertices.get(i), adjVertices.get(j)};
                    double saving = c_matrix[0][i] + c_matrix[0][j] - c_matrix[i][j];
                    savingsPairListUnsorted.put(pair, saving);
                }
            }
        }

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

        step5();
    }

    private void step5() {
//        System.out.println("Step 5");
        int numFeasRoutes = feasibleToursList.size();
        while (!savingsPairList.isEmpty()) {
            int n = feasibleToursList.size();
            int[] pair = pairList.get(0);
//            int[] pair = pairs.get(0).getKey();
//            System.out.println("Pair: " + pair[0] + ", " + pair[1]);
//            Double saving = savingsPairList.get(pair);
            savingsPairList.remove(pair);
            pairList.remove(pair);
//            pairs.remove(0);

//            ArrayList<Integer> tour_i = adjacentVertexToRoute.get(pair[0]);
//            ArrayList<Integer> tour_j = adjacentVertexToRoute.get(pair[1]);

            ArrayList<Integer> tour_i = new ArrayList<>();
            ArrayList<Integer> tour_j = new ArrayList<>();

            for (ArrayList<Integer> tour : feasibleToursList) {
                if (tour.get(1) == pair[0]) {
                    tour_i = tour;
                }
                if (tour.get(1) == pair[1]) {
                    tour_j = tour;
                }
            }

            ArrayList<Integer> mergedTour = getMergedTour(tour_i, tour_j);
            ArrayList<Integer> mergedTourReverse = getMergedTour(tour_j, tour_i);

            if (isFeasible(mergedTour)) {
                feasibleToursList.remove(tour_i); //remove also the tours that are merged
                feasibleToursList.remove(tour_j); //remove also the tours that are merged
                feasibleToursList.add(mergedTour);
            } else if (isFeasible(mergedTourReverse)) {
                feasibleToursList.remove(tour_i); //remove also the tours that are merged
                feasibleToursList.remove(tour_j); //remove also the tours that are merged
                feasibleToursList.add(mergedTourReverse);
            } else if (isTimeFeasible(mergedTour) && isTimeFeasible(mergedTourReverse)) {
                mergedTour = insertedStationTour(pair, tour_i, tour_j, mergedTour);
                mergedTourReverse = insertedStationTour(new int[]{pair[1], pair[0]}, tour_j, tour_i, mergedTourReverse);
                if (mergedTour != null & mergedTourReverse != null) {
                    ArrayList<Integer> stationsInRoute = stationsInRoute(mergedTour);
                    ArrayList<Integer> stationsInRouteReverse = stationsInRoute(mergedTourReverse);
                    if (stationsInRoute.size() > 1 && stationsInRouteReverse.size() > 1) {
                        ArrayList<Integer> removedCombinationIJ = getRedundantStations(mergedTour, stationsInRoute);
                        mergedTour.removeAll(removedCombinationIJ);
                        ArrayList<Integer> removedCombinationJI = getRedundantStations(mergedTourReverse, stationsInRouteReverse);
                        mergedTourReverse.removeAll(removedCombinationJI);
                        if (removedCombinationIJ.size() > removedCombinationJI.size()) {
                            chargingStations.addAll(removedCombinationIJ); // Added
                            feasibleToursList.remove(tour_i); //remove also the tours that are merged
                            feasibleToursList.remove(tour_j); //remove also the tours that are merged
                            feasibleToursList.add(mergedTour);
                        } else if (removedCombinationIJ.size() < removedCombinationJI.size()) {
                            chargingStations.addAll(removedCombinationJI); // Added
                            feasibleToursList.remove(tour_i); //remove also the tours that are merged
                            feasibleToursList.remove(tour_j); //remove also the tours that are merged
                            feasibleToursList.add(mergedTourReverse);
                        } else {
                            double costIJ = computeCost(mergedTour);
                            double costJI = computeCost(mergedTourReverse);
                            if (costIJ >= costJI) {
                                chargingStations.addAll(removedCombinationJI); // Added
                                feasibleToursList.remove(tour_i); //remove also the tours that are merged
                                feasibleToursList.remove(tour_j); //remove also the tours that are merged
                                feasibleToursList.add(mergedTourReverse);
                            } else {
                                chargingStations.addAll(removedCombinationIJ); // Added
                                feasibleToursList.remove(tour_i); //remove also the tours that are merged
                                feasibleToursList.remove(tour_j); //remove also the tours that are merged
                                feasibleToursList.add(mergedTour);
                            }
                        }
                    } else if (stationsInRouteReverse.size() > 1) {
                        // stationsInRoute <= 1 -> size of the tour is smaller than that of its reverse
                        feasibleToursList.remove(tour_i); //remove also the tours that are merged
                        feasibleToursList.remove(tour_j); //remove also the tours that are merged
                        feasibleToursList.add(mergedTour);
                    } else if (stationsInRoute.size() > 1) {
                        feasibleToursList.remove(tour_i); //remove also the tours that are merged
                        feasibleToursList.remove(tour_j); //remove also the tours that are merged
                        feasibleToursList.add(mergedTourReverse);
                    }

                } else if (mergedTour != null) {
                    ArrayList<Integer> stationsInRoute = stationsInRoute(mergedTour);
                    if (stationsInRoute.size() > 1) {
                        ArrayList<Integer> removedCombinationIJ = getRedundantStations(mergedTour, stationsInRoute);
                        mergedTour.removeAll(removedCombinationIJ);
                        chargingStations.addAll(removedCombinationIJ); // Added
                    }
                    feasibleToursList.remove(tour_i); //remove also the tours that are merged
                    feasibleToursList.remove(tour_j); //remove also the tours that are merged
                    feasibleToursList.add(mergedTour);
                } else if (mergedTourReverse != null) {
                    ArrayList<Integer> stationsInRoute = stationsInRoute(mergedTourReverse);
                    if (stationsInRoute.size() > 1) {
                        ArrayList<Integer> removedCombinationJI = getRedundantStations(mergedTourReverse, stationsInRoute);
                        mergedTourReverse.removeAll(removedCombinationJI);
                        chargingStations.addAll(removedCombinationJI); // Added
                    }
                    feasibleToursList.remove(tour_i); //remove also the tours that are merged
                    feasibleToursList.remove(tour_j); //remove also the tours that are merged
                    feasibleToursList.add(mergedTourReverse);
                }
            } else if (isTimeFeasible(mergedTour)) {
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
                feasibleToursList.remove(tour_i); //remove also the tours that are merged
                feasibleToursList.remove(tour_j); //remove also the tours that are merged
                feasibleToursList.add(mergedTour);
            } else if (isTimeFeasible(mergedTourReverse)) {
                mergedTourReverse = insertedStationTour(new int[]{pair[1], pair[0]}, tour_j, tour_i, mergedTourReverse);
                if (mergedTourReverse == null) {
                    break;
                }
                ArrayList<Integer> stationsInRoute = stationsInRoute(mergedTourReverse);
                if (stationsInRoute.size() > 1) {
                    ArrayList<Integer> removedCombinationJI = getRedundantStations(mergedTourReverse, stationsInRoute);
                    mergedTourReverse.removeAll(removedCombinationJI);
                    chargingStations.addAll(removedCombinationJI); // Added
                }
                feasibleToursList.remove(tour_i); //remove also the tours that are merged
                feasibleToursList.remove(tour_j); //remove also the tours that are merged
                feasibleToursList.add(mergedTourReverse);
            }
            if (n != feasibleToursList.size()) {
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
        if (feasibleToursList.size() != numFeasRoutes) {
            step4();
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

        //TODO: can be null, make sure that it doesnt call when null
        //TODO: additionally, remove the charging stations if they are used
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
            int finalBestStation = bestStation;
            chargingStations.removeIf(station -> station == finalBestStation);
            return newMergedTour;
        } else {
            return null;
        }

    }

    private ArrayList<Integer> getMergedTour(ArrayList<Integer> tour_i, ArrayList<Integer> tour_j) {
        ArrayList<Integer> mergedTour = new ArrayList<>();
//        System.out.println(tour_i);
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

        int finalMaxCombSize = maxCombSize;
        feasibleRemovals.removeIf(comb -> comb.size() < finalMaxCombSize);

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

    // Check if a location is part of the tour + return its position in the tour (return -1 if its not part of the tour)
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

    // TODO: Consider the time spent in the charging stations
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
            xi = Math.max(0, xi);
            time += t_matrix[tour.get(i)][tour.get(i + 1)];
            time += xi * xi / 100;
            //TODO: might be wrong spot
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
