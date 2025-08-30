import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.application.Application;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.*;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import javafx.util.Duration;

import java.util.*;

public class SmartCityTrafficSimulation extends Application {

    // Canvas / UI
    private static final int WIDTH = 1000;
    private static final int HEIGHT = 600;
    private static final int MARGIN = 80;

    // Visual tuning
    private static final double NODE_RADIUS = 10;
    private static final double ROAD_WIDTH = 18;
    private static final double VEHICLE_W = 14;
    private static final double VEHICLE_H = 9;

    // Simulation tuning
    private static final int FRAME_MS = 60;
    private static final int ROAD_CAPACITY = 5; // each road capacity = 5 vehicles

    private final Random rand = new Random();

    // World
    private GridGraph graph;
    private final List<Vehicle> vehicles = new ArrayList<>();
    private int simTime = 0;

    // UI
    private Timeline timeline;
    private Label lblTime, lblCongestion, lblTotal;
    private TextField tfRows, tfCols, tfCars, tfAmb, tfPolice;

    @Override
    public void start(Stage primaryStage) {
        BorderPane root = new BorderPane();

        Canvas canvas = new Canvas(WIDTH, HEIGHT);
        root.setCenter(canvas);
        GraphicsContext gc = canvas.getGraphicsContext2D();

        // Controls
        tfRows = new TextField("4"); tfRows.setPrefWidth(50);
        tfCols = new TextField("5"); tfCols.setPrefWidth(50);
        tfCars = new TextField("50"); tfCars.setPrefWidth(60);
        tfAmb = new TextField("15"); tfAmb.setPrefWidth(60);
        tfPolice = new TextField("15"); tfPolice.setPrefWidth(60);

        Button btnStart = new Button("Start");
        Button btnPause = new Button("Pause");
        Button btnReset = new Button("Reset");

        lblTime = new Label("⏱ Time: 0");
        lblCongestion = new Label("🔴 Congested Roads: 0");
        lblTotal = new Label("🚗 Total: 0");

        HBox top = new HBox(8,
                new Label("Rows:"), tfRows,
                new Label("Cols:"), tfCols,
                new Label("Cars:"), tfCars,
                new Label("Ambulances:"), tfAmb,
                new Label("Police:"), tfPolice,
                btnStart, btnPause, btnReset,
                new Separator(),
                lblTime, lblTotal, lblCongestion
        );
        top.setPadding(new Insets(8));
        top.setAlignment(Pos.CENTER_LEFT);
        root.setTop(top);

        // Legend + Smart Management box
        VBox rightBox = new VBox(8);
        rightBox.setPadding(new Insets(8));
        VBox legend = buildLegend();
        rightBox.getChildren().add(legend);
        root.setRight(rightBox);

        // Timeline
        timeline = new Timeline(new KeyFrame(Duration.millis(FRAME_MS), e -> {
            stepSimulation();
            draw(gc);
        }));
        timeline.setCycleCount(Timeline.INDEFINITE);

        btnStart.setOnAction(e -> {
            buildFromInputs();
            timeline.play();
        });
        btnPause.setOnAction(e -> timeline.pause());
        btnReset.setOnAction(e -> {
            timeline.stop();
            buildFromInputs();
            draw(gc);
        });

        // initialize
        buildFromInputs();
        draw(gc);

        Scene scene = new Scene(root);
        primaryStage.setScene(scene);
        primaryStage.setTitle("SMART CITY TRAFFIC SIMULATION");
        primaryStage.show();
    }

    private VBox buildLegend() {
        VBox v = new VBox(8);
        v.setPadding(new Insets(12));
        v.setPrefWidth(260);
        v.setStyle("-fx-background-color: rgba(255,255,255,0.95); -fx-border-color: #44444433; -fx-border-radius: 8; -fx-background-radius: 8;");
        Label title = new Label("Info");
        title.setStyle("-fx-font-weight: bold");
        v.getChildren().add(title);

        v.getChildren().add(legendItemCircle(Color.DODGERBLUE, "Node"));
        v.getChildren().add(legendItemLine(Color.DARKGRAY, "Road"));
        v.getChildren().add(legendItemRect(Color.CYAN, "Car"));
        v.getChildren().add(legendItemRect(Color.WHITE, "Ambulance"));
        v.getChildren().add(legendItemRect(Color.SLATEGRAY, "Police"));
        v.getChildren().add(legendItemRect(Color.RED, "Congested Road"));

        v.getChildren().add(new Separator());

        // Smart traffic management explanation box
        VBox smartBox = new VBox(6);
        Label smTitle = new Label("Smart Management");
        smTitle.setStyle("-fx-font-weight: bold;");
        Label sm1 = new Label("• Each road capacity = " + ROAD_CAPACITY + " vehicles");
        Label sm2 = new Label("• Vehicles avoid entering roads\n at/above capacity");
        Label sm3 = new Label("• Ambulance priority over traffic lights");
        Label sm4 = new Label("• Police semi-priority over traffic lights");
        Label sm5 = new Label("• Cars stop at RED, sometimes\n hesitate at YELLOW.");
        smartBox.getChildren().addAll(smTitle, sm1, sm2, sm3, sm4, sm5);

        v.getChildren().add(smartBox);

//        v.getChildren().add(new Separator());

        return v;
    }

    private HBox legendItemCircle(Color c, String text) {
        Canvas cc = new Canvas(24, 18);
        GraphicsContext g = cc.getGraphicsContext2D();
        g.setFill(c); g.fillOval(6,3,12,12);
        Label l = new Label(text);
        return new HBox(8, cc, l);
    }
    private HBox legendItemLine(Color c, String text) {
        Canvas cc = new Canvas(36, 18);
        GraphicsContext g = cc.getGraphicsContext2D();
        g.setStroke(c); g.setLineWidth(6); g.strokeLine(4,9,32,9);
        Label l = new Label(text);
        return new HBox(8, cc, l);
    }
    private HBox legendItemRect(Color c, String text) {
        Canvas cc = new Canvas(36, 18);
        GraphicsContext g = cc.getGraphicsContext2D();
        g.setFill(c); g.fillRoundRect(6,5,20,8,4,4);
        g.setStroke(Color.BLACK); g.strokeRoundRect(6,5,20,8,4,4);
        Label l = new Label(text);
        return new HBox(8, cc, l);
    }

    private void buildFromInputs() {
        int rows = clamp(parseInt(tfRows.getText(), 4), 2, 20);
        int cols = clamp(parseInt(tfCols.getText(), 4), 2, 20);
        int cars = clamp(parseInt(tfCars.getText(), 15), 0, 500);
        int amb = clamp(parseInt(tfAmb.getText(), 5), 0, 100);
        int pol = clamp(parseInt(tfPolice.getText(), 5), 0, 100);

        graph = GridGraph.generateGrid(rows, cols, WIDTH, HEIGHT, MARGIN, rand);
        vehicles.clear();
        generateVehiclesOnEdges(cars, amb, pol);
        simTime = 0;
        updateLabels();
    }

    /**
     * Place vehicles ON random edges so they already follow roads (not at nodes).
     * Try to respect road capacity when spawning; if none available, fallback.
     */
    private void generateVehiclesOnEdges(int cars, int amb, int police) {
        int total = cars + amb + police;
        List<GridGraph.Edge> candidateEdges = new ArrayList<>(graph.edges());
        if (candidateEdges.isEmpty()) return;

        for (int i = 0; i < total; i++) {
            VehicleType type;
            if (i < amb) type = VehicleType.AMBULANCE;
            else if (i < amb + police) type = VehicleType.POLICE;
            else type = VehicleType.CAR;

            // choose random road trying to respect capacity
            GridGraph.Edge e = null;
            List<GridGraph.Edge> under = new ArrayList<>();
            for (GridGraph.Edge edge : candidateEdges) if (edge.load < ROAD_CAPACITY) under.add(edge);
            if (!under.isEmpty()) e = under.get(rand.nextInt(under.size()));
            else e = candidateEdges.get(rand.nextInt(candidateEdges.size()));

            boolean dirForward = rand.nextBoolean();
            int fromNode = dirForward ? e.u : e.v;
            int toNode = dirForward ? e.v : e.u;
            double progress = rand.nextDouble() * 0.8;
            Vehicle v = new Vehicle(i, fromNode, toNode, type);
            v.onEdge = e;
            v.edgeProgress = progress;
            v.currentSpeed = v.baseSpeed();
            v.heading = Math.atan2(e.y2 - e.y1, e.x2 - e.x1);
            v.lastNode = fromNode;
            v.targetNode = toNode;

            e.load++;
            vehicles.add(v);
        }
    }

    private void stepSimulation() {
        simTime++;

        // update lights
        for (GridGraph.Node n : graph.nodes) {
            n.light.update();
        }

        // move vehicles
        for (Vehicle v : vehicles) {
            // if not on an edge (sitting at node), try to pick a non-congested neighbor
            if (v.onEdge == null) {
                GridGraph.Node cur = graph.node(v.currentNode);
                if (!cur.neighbors.isEmpty()) {
                    // only choose edges under capacity
                    List<GridGraph.Neighbor> options = new ArrayList<>();
                    for (GridGraph.Neighbor nb : cur.neighbors) {
                        if (nb.edge.load < ROAD_CAPACITY) {
                            // avoid immediate U-turn if possible
                            if (v.lastNode != -1 && nb.to == v.lastNode && cur.neighbors.size() > 1) continue;
                            options.add(nb);
                        }
                    }
                    if (options.isEmpty()) {
                        // no available road under capacity -> wait
                        v.stopped = true;
                        continue;
                    }
                    GridGraph.Neighbor nb = options.get(rand.nextInt(options.size()));
                    v.onEdge = nb.edge;
                    v.edgeProgress = 0.0;
                    v.currentSpeed = v.baseSpeed();
                    v.heading = Math.atan2(v.onEdge.y2 - v.onEdge.y1, v.onEdge.x2 - v.onEdge.x1);
                    v.lastNode = v.currentNode;
                    v.targetNode = nb.to;
                    v.onEdge.load++;
                    v.stopped = false;
                }
                continue;
            }

            // traveling on edge
            double moveStep = v.currentSpeed;

            if (v.edgeProgress < 1e-6) {
                // we are about to leave node v.lastNode -> check its light
                GridGraph.Node fromNode = graph.node(v.lastNode);
                if (!fromNode.light.allows(v, rand)) {
                    v.stopped = true;
                    continue; // wait
                } else {
                    v.stopped = false;
                }
            }

            if (v.onEdge.load >= ROAD_CAPACITY) {
                if (v.type == VehicleType.CAR) {
                    moveStep = v.baseSpeed() * 0.4;  // cars slow down more
                } else if (v.type == VehicleType.POLICE) {
                    moveStep = v.baseSpeed() * 0.7;  // police slow less
                } else if (v.type == VehicleType.AMBULANCE) {
                    moveStep = v.baseSpeed() * 0.9;  // ambulance almost unaffected
                }
            }
            
            v.edgeProgress += moveStep;

            // bound progress
            if (v.edgeProgress >= 1.0) {
                // arrived at the end of edge
                int arrived = (v.lastNode == v.onEdge.u) ? v.onEdge.v : v.onEdge.u;

                // decrement load on that edge
                v.onEdge.load = Math.max(0, v.onEdge.load - 1);

                // update node state
                v.currentNode = arrived;
                v.onEdge = null;
                v.edgeProgress = 0.0;

                // choose next edge from arrived node
                GridGraph.Node node = graph.node(arrived);
                List<GridGraph.Neighbor> choices = new ArrayList<>(node.neighbors);

                // avoid U-turn
                if (v.lastNode != -1 && choices.size() > 1) {
                    choices.removeIf(nb -> nb.to == v.lastNode);
                }

                // filter only edges under capacity
                choices.removeIf(nb -> nb.edge.load >= ROAD_CAPACITY);

                if (choices.isEmpty()) {
                    // no available non-congested outgoing edge -> wait at node
                    v.stopped = true;
                    v.lastNode = arrived;
                    continue;
                }

                // pick next neighbor randomly
                GridGraph.Neighbor next = choices.get(rand.nextInt(choices.size()));
                // set new edge and progress to 0
                v.onEdge = next.edge;
                // determine direction: if leaving arrived -> to == neighbor.to
                // need to ensure edge.direction aligns with travel: if edge.u == arrived then travel u->v else v->u
                v.lastNode = arrived;
                v.targetNode = next.to;
                v.edgeProgress = 0.0;
                v.onEdge.load++;

                // compute new heading target and apply turning slowdown if angle large
                double newHeading = Math.atan2(v.onEdge.y2 - v.onEdge.y1, v.onEdge.x2 - v.onEdge.x1);
                // if traveling reverse on the physical edge (we may need opposite direction), ensure heading matches travel direction
                if (v.lastNode == v.onEdge.v) {
                    // traveling from v to u -> invert
                    newHeading = Math.atan2(v.onEdge.y1 - v.onEdge.y2, v.onEdge.x1 - v.onEdge.x2);
                }

                double angleDelta = smallestAngleDiff(v.heading, newHeading);
                double absDeg = Math.abs(Math.toDegrees(angleDelta));
                if (absDeg > 20) {
                    // slow down for the first portion of the edge to simulate turning
                    v.currentSpeed = v.baseSpeed() * 0.5;
                } else {
                    v.currentSpeed = v.baseSpeed();
                }
                // set heading towards newHeading immediately (we rotate smoothly during draw)
                v.heading = newHeading;
                v.stopped = false;
            } else {
                // while traveling, if approaching a yellow light at the end node, slow a bit
                int toNode = (v.lastNode == v.onEdge.u) ? v.onEdge.v : v.onEdge.u;
                GridGraph.Node toN = graph.node(toNode);
                if (toN.light.state == TrafficLight.State.YELLOW && v.type == VehicleType.CAR) {
                    if (1.0 - v.edgeProgress < 0.25) {
                        v.currentSpeed = v.baseSpeed() * 0.6;
                    }
                }
            }
        }

        // after moving all vehicles, update labels
        updateLabels();
    }

    private void draw(GraphicsContext gc) {
        // background
        gc.setFill(Color.web("#0b1020"));
        gc.fillRect(0, 0, WIDTH, HEIGHT);

        int congested = 0;

        // draw roads first (so nodes are drawn over them)
        for (GridGraph.Edge e : graph.edges()) {
            if (e.load >= ROAD_CAPACITY) {
                gc.setStroke(Color.RED);
                congested++;
            } else {
                gc.setStroke(Color.DARKGRAY);
            }
            gc.setLineWidth(ROAD_WIDTH);
            gc.strokeLine(e.x1, e.y1, e.x2, e.y2);

            // center dashed line
            gc.setLineWidth(2);
            gc.setStroke(Color.WHITE);
            gc.setLineDashes(12);
            gc.strokeLine(e.x1, e.y1, e.x2, e.y2);
            gc.setLineDashes(0);

            // show load near middle (L:current/capacity)
            gc.setFill(Color.WHITE);
            double cx = (e.x1 + e.x2) / 2.0;
            double cy = (e.y1 + e.y2) / 2.0 - 14;
            gc.fillText("L:" + e.load + "/" + ROAD_CAPACITY, cx - 18, cy);
        }

        // draw nodes
        for (GridGraph.Node n : graph.nodes) {
            gc.setFill(Color.DODGERBLUE);
            gc.fillOval(n.x - NODE_RADIUS, n.y - NODE_RADIUS, NODE_RADIUS * 2, NODE_RADIUS * 2);
            gc.setFill(Color.WHITE);
            gc.fillText("N" + n.id, n.x - 9, n.y + 4);
        }

        // draw traffic lights
        for (GridGraph.Node n : graph.nodes) {
            double ox = n.x + 15; // diagonal offset
            double oy = n.y - 40;
            gc.setFill(Color.WHITE);
            gc.fillRoundRect(ox - 4, oy - 8, 18, 36, 6, 6);

            if (n.light.state == TrafficLight.State.GREEN) {
                gc.setFill(Color.LIME);
                gc.fillOval(ox, oy - 6, 10, 10);
            } else if (n.light.state == TrafficLight.State.YELLOW) {
                gc.setFill(Color.GOLD);
                gc.fillOval(ox, oy + 4, 10, 10);
            } else {
                gc.setFill(Color.RED);
                gc.fillOval(ox, oy + 14, 10, 10);
            }
        }

        // draw vehicles: compute current position along their edge or at node
        for (Vehicle v : vehicles) {
            double vx, vy;
            double heading = v.heading;

            if (v.onEdge == null) {
                // if sitting at node
                GridGraph.Node n = graph.node(v.currentNode);
                vx = n.x; vy = n.y;
            } else {
                // compute traveling direction according to which direction we are going
                GridGraph.Edge e = v.onEdge;
                double x1 = e.x1, y1 = e.y1, x2 = e.x2, y2 = e.y2;
                boolean forward = (v.lastNode == e.u);
                double px = forward ? (1 - v.edgeProgress) * x1 + v.edgeProgress * x2
                                    : (1 - v.edgeProgress) * x2 + v.edgeProgress * x1;
                double py = forward ? (1 - v.edgeProgress) * y1 + v.edgeProgress * y2
                                    : (1 - v.edgeProgress) * y2 + v.edgeProgress * y1;
                vx = px; vy = py;

                // compute heading based on travel direction to draw rotated rectangle
                heading = forward ? Math.atan2(y2 - y1, x2 - x1) : Math.atan2(y1 - y2, x1 - x2);

                // Smooth rotation interpolation for nicer visual turning:
                double diff = smallestAngleDiff(v.displayHeading, heading);
                double maxStep = Math.toRadians(8);
                if (Math.abs(diff) > maxStep) {
                    v.displayHeading += Math.signum(diff) * maxStep;
                } else {
                    v.displayHeading = heading;
                }
            }

            // color by type
            if (v.type == VehicleType.CAR) gc.setFill(Color.CYAN);
            else if (v.type == VehicleType.AMBULANCE) gc.setFill(Color.WHITE);
            else gc.setFill(Color.SLATEGRAY);

            // draw rotated vehicle
            gc.save();
            gc.translate(vx, vy);
            gc.rotate(Math.toDegrees(v.displayHeading));
            gc.fillRoundRect(-VEHICLE_W / 2.0, -VEHICLE_H / 2.0, VEHICLE_W, VEHICLE_H, 6, 6);
            gc.setStroke(Color.BLACK); gc.strokeRoundRect(-VEHICLE_W / 2.0, -VEHICLE_H / 2.0, VEHICLE_W, VEHICLE_H, 6, 6);
            gc.restore();

            // label
            gc.setFill(Color.BLACK);
            String lab = (v.type == VehicleType.CAR) ? "Car" : (v.type == VehicleType.AMBULANCE) ? "Amb" : "Police";
            gc.fillText(lab, vx - 12, vy - 12);

            if (v.stopped) {
                gc.setFill(Color.WHITE);
                gc.fillText("Stopped", vx - 60, vy - 22);
            }
        }

        lblCongestion.setText("🔴 Congested Roads: " + congested);
    }

    private void updateLabels() {
        lblTime.setText("⏱ Time: " + simTime);
        lblTotal.setText("🚗 Total: " + vehicles.size());
    }

    // ---------------- utilities ----------------
    private int parseInt(String s, int def) {
        try { return Integer.parseInt(s.trim()); } catch (Exception ex) { return def; }
    }
    private int clamp(int v, int lo, int hi) { return Math.max(lo, Math.min(hi, v)); }

    /**
     * Smallest signed angle difference, result in [-PI, PI]
     */
    private static double smallestAngleDiff(double from, double to) {
        double a = to - from;
        while (a <= -Math.PI) a += 2 * Math.PI;
        while (a > Math.PI) a -= 2 * Math.PI;
        return a;
    }

    // ---------------- Graph structures ----------------
    static class GridGraph {
        final int rows, cols;
        final Node[] nodes;
        final List<Edge> edges = new ArrayList<>();
        final List<List<Neighbor>> adj;

        GridGraph(int rows, int cols) {
            this.rows = rows; this.cols = cols;
            nodes = new Node[rows * cols];
            adj = new ArrayList<>(rows * cols);
            for (int i = 0; i < rows * cols; i++) adj.add(new ArrayList<>());
        }

        static GridGraph generateGrid(int rows, int cols, int width, int height, int margin, Random rand) {
            GridGraph g = new GridGraph(rows, cols);
            double usableW = width - 2.0 * margin;
            double usableH = height - 2.0 * margin;
            for (int r = 0; r < rows; r++) {
                for (int c = 0; c < cols; c++) {
                    int id = r * cols + c;
                    double x = margin + (usableW) * (c / (double) Math.max(1, cols - 1));
                    double y = margin + (usableH) * (r / (double) Math.max(1, rows - 1));
                    g.nodes[id] = new Node(id, (int) Math.round(x), (int) Math.round(y), new TrafficLight(rand));
                }
            }
            // edges horizontally and vertically
            for (int r = 0; r < rows; r++) {
                for (int c = 0; c < cols; c++) {
                    int id = r * cols + c;
                    if (c + 1 < cols) g.addEdgeUnique(id, id + 1);
                    if (r + 1 < rows) g.addEdgeUnique(id, id + cols);
                }
            }
            return g;
        }

        void addEdgeUnique(int a, int b) {
            if (a == b) return;
            if (hasEdge(a, b)) return;
            Node na = nodes[a]; Node nb = nodes[b];
            Edge e = new Edge(a, b, na.x, na.y, nb.x, nb.y, ROAD_CAPACITY);
            edges.add(e);
            adj.get(a).add(new Neighbor(b, e));
            adj.get(b).add(new Neighbor(a, e));
            // register neighbor references on nodes for convenience
            nodes[a].neighbors.add(new Neighbor(b, e));
            nodes[b].neighbors.add(new Neighbor(a, e));
        }

        boolean hasEdge(int a, int b) {
            for (Neighbor nb : adj.get(a)) if (nb.to == b) return true;
            return false;
        }

        Node node(int i) { return nodes[i]; }
        List<Edge> edges() { return edges; }

        static class Edge {
            final int u, v;
            final double x1, y1, x2, y2;
            int load = 0;
            final int capacity;
            Edge(int u, int v, double x1, double y1, double x2, double y2, int capacity) {
                this.u = u; this.v = v; this.x1 = x1; this.y1 = y1; this.x2 = x2; this.y2 = y2;
                this.capacity = capacity;
            }
        }

        static class Node {
            final int id, x, y;
            final TrafficLight light;
            final List<Neighbor> neighbors = new ArrayList<>();
            Node(int id, int x, int y, TrafficLight light) { this.id = id; this.x = x; this.y = y; this.light = light; }
        }

        static class Neighbor {
            final int to; final Edge edge;
            Neighbor(int to, Edge edge) { this.to = to; this.edge = edge; }
        }
    }

    // ---------------- Traffic Light ----------------
    static class TrafficLight {
        enum State { GREEN, YELLOW, RED }
        State state = State.GREEN;
        int timer = 0;
        final int offset;
        final int greenDur, yellowDur, redDur;

        TrafficLight(Random rand) {
            this.greenDur = 60 + rand.nextInt(40);   // frames
            this.yellowDur = 12 + rand.nextInt(8);
            this.redDur = 70 + rand.nextInt(50);
            this.offset = rand.nextInt(1000);
        }

        void update() {
            timer++;
            int cycle = (timer + offset) % (greenDur + yellowDur + redDur);
            if (cycle < greenDur) state = State.GREEN;
            else if (cycle < greenDur + yellowDur) state = State.YELLOW;
            else state = State.RED;
        }

        boolean allows(Vehicle v, Random rand) {
            if (v.type == VehicleType.AMBULANCE || v.type == VehicleType.POLICE) return true;
            if (state == State.GREEN) return true;
            if (state == State.RED) return false;
            // YELLOW: cautious cars may stop
            return rand.nextDouble() < 0.6;
        }
    }

    // ---------------- Vehicle ----------------
    enum VehicleType { CAR, AMBULANCE, POLICE }

    static class Vehicle {
        final int id;
        int currentNode;          // current node when not on edge
        int lastNode = -1;        // the node we came from (needed to pick next)
        int targetNode = -1;      // the node we're heading to (useful)
        GridGraph.Edge onEdge = null;
        double edgeProgress = 0.0; // 0..1 along edge in travel direction
        double currentSpeed = 0.0;
        double heading = 0.0;      // actual logical heading for travel
        double displayHeading = 0.0; // used for smooth visual rotation
        final VehicleType type;
        boolean stopped = false;

        Vehicle(int id, int startNode, int destNode, VehicleType type) {
            this.id = id;
            this.currentNode = startNode;
            this.targetNode = destNode;
            this.type = type;
            this.displayHeading = 0.0;
        }

        double baseSpeed() {
            // tuned to FRAME_MS; adjust for smoother visuals
            if (type == VehicleType.AMBULANCE) return 0.015; // faster
            if (type == VehicleType.POLICE) return 0.012;
            return 0.010; // cars slower
        }
    }

    private void generateVehicles(int cars, int amb, int police) {
        // not used in this file; kept if you later want to spawn at nodes
    }

    public static void main(String[] args) {
        launch(args);
    }
}
