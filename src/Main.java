import java.util.*;
import java.util.concurrent.*;

class Node {
    String name;
    boolean visited;
    int ttl;

    public Node(String name, int ttl) {
        this.name = name;
        this.visited = false;
        this.ttl = ttl;
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

    public List<String> parallelAStarSearch(String start, String goal) throws InterruptedException {
        ExecutorService executor = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
        PriorityQueue<NodeEntry> openSet = new PriorityQueue<>(Comparator.comparingInt(ne -> ne.fScore));
        Map<Node, Node> cameFrom = new ConcurrentHashMap<>();
        Map<Node, Integer> gScore = new ConcurrentHashMap<>();
        Map<Node, Integer> fScore = new ConcurrentHashMap<>();
        Map<Node, Integer> hopCount = new ConcurrentHashMap<>();

        Node startNode = nodes.get(start);
        Node goalNode = nodes.get(goal);

        if (startNode == null || goalNode == null) return new ArrayList<>();

        openSet.add(new NodeEntry(startNode, 0, heuristic(startNode, goal)));
        gScore.put(startNode, 0);
        fScore.put(startNode, heuristic(startNode, goal));
        hopCount.put(startNode, 0);

        while (!openSet.isEmpty()) {
            NodeEntry currentEntry = openSet.poll();
            Node current = currentEntry.node;
            current.visited = true;

            if (current == goalNode) {
                executor.shutdown();
                return reconstructPath(cameFrom, goalNode, hopCount.get(goalNode));
            }

            List<Edge> neighbors = adjacencyList.getOrDefault(current, new ArrayList<>());
            List<Runnable> tasks = new ArrayList<>();

            for (Edge edge : neighbors) {
                Node neighbor = edge.to;
                if (neighbor.visited || neighbor.ttl <= 0) continue;

                tasks.add(() -> {
                    int tentativeGScore = gScore.getOrDefault(current, Integer.MAX_VALUE) + edge.cost;
                    if (tentativeGScore < gScore.getOrDefault(neighbor, Integer.MAX_VALUE)) {
                        cameFrom.put(neighbor, current);
                        gScore.put(neighbor, tentativeGScore);
                        fScore.put(neighbor, tentativeGScore + heuristic(neighbor, goal));
                        hopCount.put(neighbor, hopCount.getOrDefault(current, 0) + 1);
                        openSet.add(new NodeEntry(neighbor, tentativeGScore, fScore.get(neighbor)));
                    }
                });
            }

            tasks.forEach(executor::submit);
        }
        executor.shutdown();
        return new ArrayList<>();
    }

    private int heuristic(Node node, String goal) {
        return node.name.equals(goal) ? 0 : node.name.length();
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
        Graph graph = new Graph();
        graph.addNode("A", 3);
        graph.addNode("B", 2);
        graph.addNode("C", 5);
        graph.addNode("D", 1);

        graph.addEdge("A", "B", 1);
        graph.addEdge("B", "C", 2);
        graph.addEdge("C", "D", 3);
        graph.addEdge("A", "D", 4);

        System.out.println(graph.parallelAStarSearch("A", "C"));
        System.out.println(graph.parallelAStarSearch("A", "B"));
        System.out.println(graph.parallelAStarSearch("B", "D"));
        System.out.println(graph.parallelAStarSearch("D", "B"));
    }
}