import java.util.*;
import java.util.concurrent.*;


/*
The A* (A-star) search algorithm is a popular pathfinding and graph traversal algorithm.
It finds the shortest path from a start node to a goal node using a best-first search approach.
A* combines the advantages of Dijkstra’s Algorithm and Greedy Best-First Search
by using both the actual cost to reach a node and a heuristic estimate of the cost to reach the goal.
 */
class Node {
    String name;
    boolean visited;
    int ttl;

    public Node(String name, int ttl) {
        this.name = name;
        this.visited = false;
        this.ttl = ttl; //index from start position.
    }
}

class Graph {
    Map<String, Node> nodes;
    Map<Node, List<Edge>> adjacencyList;

    public Graph() {
        nodes = new HashMap<>();
        adjacencyList = new HashMap<>();
    }

    public void addNode(String name, int ttl) {
        Node node = new Node(name, ttl);
        nodes.put(name, node);
        adjacencyList.put(node, new ArrayList<>());
    }

    public void addEdge(String from, String to, int cost) {
        Node nodeFrom = nodes.get(from);
        Node nodeTo = nodes.get(to);
        if (nodeFrom != null && nodeTo != null) {
            adjacencyList.get(nodeFrom).add(new Edge(nodeFrom, nodeTo, cost));
            adjacencyList.get(nodeTo).add(new Edge(nodeTo, nodeFrom, cost));
        }
    }

    private void resetNodes(){
        for (Node node : this.nodes.values()) {
            node.visited = false;
        }
    }

    public List<String> parallelAStarSearch(String start, String goal) throws InterruptedException {

        if (start.equals(goal)) { return Collections.singletonList(start);}//Check if the start and goal are the same, if so return the start node as the path
        this.resetNodes(); //All "visited" flags in each node must be reset before each search

        ExecutorService executor = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());

        PriorityQueue<NodeEntry> openSet = new PriorityQueue<>(Comparator.comparingInt(ne -> ne.fScore)); //manage the open set of nodes to explore, sorted by fScore

        Map<Node, Node> cameFrom = new ConcurrentHashMap<>();
        Map<Node, Integer> gScore = new ConcurrentHashMap<>(); //The cost from the start node to the current node.
        Map<Node, Integer> fScore = new ConcurrentHashMap<>(); //The estimated total cost from start to goal through the current node (fScore = gScore + heuristic).
        Map<Node, Integer> hopCount = new ConcurrentHashMap<>();

        Node startNode = nodes.get(start);
        Node goalNode = nodes.get(goal);

        if (startNode == null || goalNode == null) return new ArrayList<>();

        openSet.add(new NodeEntry(startNode, 0, heuristic(startNode, goal)));
        gScore.put(startNode, 0);
        fScore.put(startNode, heuristic(startNode, goal));
        hopCount.put(startNode, 0);

        //2 Stages, 1. Gathering neighbors and updating scores. 2. RUNNING SEARCH IN PARALLEL
        while (!openSet.isEmpty()) {
            NodeEntry currentEntry = openSet.poll();
            Node current = currentEntry.node;
            current.visited = true;

            if (current == goalNode) { executor.shutdown(); return reconstructPath(cameFrom, goalNode, hopCount.get(goalNode)); } //If the current node is the goal, reconstruct and return the path

            List<Edge> neighbors = adjacencyList.getOrDefault(current, new ArrayList<>());
            List<Runnable> tasks = new ArrayList<>();

            // Iterate through neighbors and update scores in parallel
            for (Edge edge : neighbors) {
                Node neighbor = edge.to;
                if (neighbor.visited || neighbor.ttl <= 0)
                    continue;

                tasks.add(() -> {
                    int tentativeGScore = gScore.getOrDefault(current, Integer.MAX_VALUE) + edge.cost;
                    if (tentativeGScore < gScore.getOrDefault(neighbor, Integer.MAX_VALUE)) {
                        synchronized (cameFrom) { //concurrent search
                            cameFrom.put(neighbor, current);
                        }
                        gScore.put(neighbor, tentativeGScore);
                        fScore.put(neighbor, tentativeGScore + heuristic(neighbor, goal));
                        hopCount.put(neighbor, hopCount.getOrDefault(current, 0) + 1);
                        synchronized (openSet) {
                            openSet.add(new NodeEntry(neighbor, tentativeGScore, fScore.get(neighbor)));
                        }
                    }
                });
            }//End For Loop

            tasks.forEach(executor::submit); //RUNS THE SEARCH IN PARALLEL
            List<Future<?>> futures = new ArrayList<>();
            for (Runnable task : tasks) {
                futures.add(executor.submit(task));
            }
            for (Future<?> future : futures) {
                try {
                    future.get(); // Wait for all neighbor updates to finish
                } catch (ExecutionException e) {
                    throw new RuntimeException(e);
                }
            }

        }//End While Loop
        executor.shutdown();
        return new ArrayList<>();
    }
    //Is a helper function that calculates the heuristic value for a node. Heuristic: An estimate of the cost from the current node to the goal (e.g., straight-line distance)
    // if goal returns 0, otherwise returns the ttl of the node as a placeholder value.
    private int heuristic(Node node, String goal) {
        return node.name.equals(goal) ? 0 : node.ttl; ////returns 0 if the node is the goal otherwise it returns the ttl(a placeholder value for now)
}

    private List<String> reconstructPath(Map<Node, Node> cameFrom, Node current, int hops) {
        List<String> path = new LinkedList<>();
        while (current != null) {
            path.add(0, current.name);
            current = cameFrom.get(current);
        }
        path.add("Total Hops: " + hops);
        return path;
    }
}

class Edge {
    Node from, to;
    int cost;

    public Edge(Node from, Node to, int cost) {
        this.from = from;
        this.to = to;
        this.cost = cost;
    }
}

class NodeEntry {
    Node node;
    int gScore;
    int fScore;

    public NodeEntry(Node node, int gScore, int fScore) {
        this.node = node;
        this.gScore = gScore;
        this.fScore = fScore;
    }
}

public class Main {
    public static void main(String[] args) throws InterruptedException {

       // test1();

        Graph graph = new Graph();
        int iFunny = 1; //is ttl.
        for(char a = 'A'; a < 'Z'; a++){
            graph.addNode(String.valueOf(a),iFunny);
            iFunny++ ;
        }

        int cost= 1;
        //Add edges
        for(char a = 'A'; a < 'Z'; a++){
            char b = (char) (a+1);
            graph.addEdge(String.valueOf(a), String.valueOf(b), cost);
            System.out.println("Value of a = " + a + "\t Value of improperly adding a char = " + a+1 + "\t and value of b = " + b);
            b = (char) (a+2);
            graph.addEdge(String.valueOf(a), String.valueOf(b), cost+1);
            b = (char) (a+3);
            graph.addEdge(String.valueOf(a), String.valueOf(b), cost+2);
        }

        for(char a = 'A'; a < 'Z'; a++){
            char b = (char) (a+13);
            graph.addEdge(String.valueOf(a), String.valueOf(b), cost);
            System.out.println("Value of a = " + a + "\t Value of improperly adding a char = " + a+1 + "\t and value of b = " + b);
        }
        for(char a = 'A'; a < 'Z'; a++){


            char b = (char) (a+10);
            if(a < 'D')         b = (char) (a+23);
            else if(a < 'H')    b = (char) (a+13);
            else if(a < 'L')    b = (char) (a+9);
            else if(a < 'O')    b = (char) (a-9);
            else if(a < 'T')    b = (char) (a-13);
            else                b = (char) (a-19);


            graph.addEdge(String.valueOf(a), String.valueOf(b), cost);
            System.out.println("Value of a = " + a + "\t Value of improperly adding a char = " + a+1 + "\t and value of b = " + b);
        }
        System.out.println(graph.parallelAStarSearch("A", "C"));
        System.out.println(graph.parallelAStarSearch("A", "B"));
        System.out.println(graph.parallelAStarSearch("B", "D"));
        System.out.println(graph.parallelAStarSearch("D", "B"));
        System.out.println(graph.parallelAStarSearch("D", "D"));

        System.out.println(graph.parallelAStarSearch("D", "Q"));
        System.out.println(graph.parallelAStarSearch("R", "A"));
        System.out.println(graph.parallelAStarSearch("A", "Z"));
        System.out.println(graph.parallelAStarSearch("B", "D"));
    }

    private static void test1() throws InterruptedException {
        Graph graph = new Graph();
        graph.addNode("A", 3);
        graph.addNode("B", 2);
        graph.addNode("C", 5);
        graph.addNode("D", 4);

        graph.addEdge("A", "B", 1);
        graph.addEdge("B", "C", 2);
        graph.addEdge("C", "D", 3);
        graph.addEdge("A", "D", 4);

        System.out.println(graph.parallelAStarSearch("A", "C"));
        System.out.println(graph.parallelAStarSearch("A", "B"));
        System.out.println(graph.parallelAStarSearch("B", "D"));
        System.out.println(graph.parallelAStarSearch("D", "B"));
        System.out.println(graph.parallelAStarSearch("D", "D"));


    }
}
