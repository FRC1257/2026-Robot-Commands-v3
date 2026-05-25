// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.utility;
 
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
 
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
 
/**
 * Generates a Mermaid {@code stateDiagram-v2} visualization of a {@link StateMachine}.
 *
 * <p>The generation runs in two stages:
 * <ol>
 *   <li>{@link #buildAst} reflects over a {@code StateMachine} and produces an in-memory
 *       tree of plain data ({@link Ast}, {@link StateNode}, {@link Edge}). That tree is the
 *       AST.</li>
 *   <li>{@link #render} walks the AST and emits Mermaid source.</li>
 * </ol>
 *
 * <p>Splitting the pipeline this way keeps reflection in one place, makes the IR
 * inspectable from tests, and leaves the back-end pluggable (Graphviz, PlantUML, etc.).
 *
 * <p>Usage:
 * <pre>{@code
 * StateMachine sm = ...;          // fully built; setInitialState already called
 * String mermaid = Multiverse.toMermaid(sm);
 * }</pre>
 *
 * <p>Condition labels default to ordinal placeholders ({@code t0}, {@code c0}, ...). For
 * meaningful labels, register the conditions you care about:
 * <pre>{@code
 * BooleanSupplier hasNote = () -> sensor.get();
 * Map<BooleanSupplier, String> labels = Map.of(hasNote, "hasNote");
 * String mermaid = Multiverse.toMermaid(sm, ConditionLabeler.from(labels));
 * }</pre>
 */
public final class Multiverse {
    private Multiverse() {}
 
    // ---------------------------------------------------------------- API
 
    /** Builds the AST with default labels, then renders to Mermaid. */
    public static String toMermaid(StateMachine sm) {
        return render(buildAst(sm));
    }
 
    public static String toMermaid(StateMachine sm, ConditionLabeler labeler) {
        return render(buildAst(sm, labeler));
    }
 
    // ---------------------------------------------------------------- AST
 
    /** Root of the AST. */
    public static final class Ast {
        public final String name;
        /** The state entered when the machine starts. {@code null} if {@code setInitialState} was never called. */
        public final StateNode initial;
        /** Declaration order, mirroring {@code StateMachine.m_states}. */
        public final List<StateNode> states;
 
        Ast(String name, StateNode initial, List<StateNode> states) {
            this.name = name;
            this.initial = initial;
            this.states = List.copyOf(states);
        }
    }
 
    /**
     * A single state node. Outgoing edges are partitioned by the builder method that
     * created them, since {@code .when(...)}, {@code .whenCompleteAnd(...)}, and
     * {@code .whenComplete()} have different evaluation semantics at runtime.
     */
    public static final class StateNode {
        public final String id;            // sanitized, mermaid-safe, unique within the AST
        public final String label;         // original command name
        public final int enterCallbackCount;
        public final int exitCallbackCount;
 
        // Populated during build, exposed read-only via accessors.
        final List<Edge> whenEdges = new ArrayList<>();
        final List<Edge> whenCompleteAndEdges = new ArrayList<>();
        Edge defaultCompletion;            // always non-null after build
 
        StateNode(String id, String label, int enterCb, int exitCb) {
            this.id = id;
            this.label = label;
            this.enterCallbackCount = enterCb;
            this.exitCallbackCount = exitCb;
        }
 
        /** Transitions registered via {@code .when(...)} — polled every scheduler loop. */
        public List<Edge> whenEdges() {
            return Collections.unmodifiableList(whenEdges);
        }
 
        /** Transitions registered via {@code .whenCompleteAnd(...)} — polled on completion. */
        public List<Edge> whenCompleteAndEdges() {
            return Collections.unmodifiableList(whenCompleteAndEdges);
        }
 
        /** Fallback completion edge — either the {@code .whenComplete()} target or the implicit exit. */
        public Edge defaultCompletion() {
            return defaultCompletion;
        }
    }
 
    /** An outgoing edge. {@code target == null} means "exit the state machine" ({@code [*]} in Mermaid). */
    public static final class Edge {
        public enum Kind { WHEN, WHEN_COMPLETE_AND, WHEN_COMPLETE_DEFAULT }
 
        public final Kind kind;
        public final StateNode target;
        /** Human-readable label for the trigger condition. {@code null} for {@link Kind#WHEN_COMPLETE_DEFAULT}. */
        public final String conditionLabel;
        /** True if the target supplier is user-provided (i.e. the destination can vary at runtime). */
        public final boolean dynamic;
        /** 0-based priority within the originating state's list of edges of the same kind. */
        public final int order;
 
        Edge(Kind kind, StateNode target, String conditionLabel, boolean dynamic, int order) {
            this.kind = kind;
            this.target = target;
            this.conditionLabel = conditionLabel;
            this.dynamic = dynamic;
            this.order = order;
        }
    }
 
    // ---------------------------------------------------------------- Builder
 
    public static Ast buildAst(StateMachine sm) {
        return buildAst(sm, ConditionLabeler.byOrder());
    }
 
    public static Ast buildAst(StateMachine sm, ConditionLabeler labeler) {
        try {
            String name = (String) readField(StateMachine.class, "m_name").get(sm);
            @SuppressWarnings("unchecked")
            List<State> rawStates = (List<State>) readField(StateMachine.class, "m_states").get(sm);
            State initialRaw = (State) readField(StateMachine.class, "m_initialState").get(sm);
 
            // Pass 1: allocate StateNode shells with stable IDs so edges can reference them.
            Map<State, StateNode> nodeFor = new IdentityHashMap<>();
            Map<String, Integer> idCounts = new HashMap<>();
            List<StateNode> ordered = new ArrayList<>(rawStates.size());
 
            for (State s : rawStates) {
                Command cmd = (Command) readField(State.class, "m_command").get(s);
                String label = cmd.name();
                String id = uniqueId(label, idCounts);
 
                @SuppressWarnings("unchecked")
                List<Runnable> enter = (List<Runnable>) readField(State.class, "m_enterCallbacks").get(s);
                @SuppressWarnings("unchecked")
                List<Runnable> exit  = (List<Runnable>) readField(State.class, "m_exitCallbacks").get(s);
 
                StateNode node = new StateNode(id, label, enter.size(), exit.size());
                nodeFor.put(s, node);
                ordered.add(node);
            }
 
            // Pass 2: populate edges. Now that every State has a StateNode shell, edges resolve cleanly.
            for (State s : rawStates) {
                StateNode node = nodeFor.get(s);
                populateWhenEdges(s, node, nodeFor, labeler);
                populateWhenCompleteAndEdges(s, node, nodeFor, labeler);
                populateDefaultCompletion(s, node, nodeFor);
            }
 
            StateNode initial = initialRaw == null ? null : nodeFor.get(initialRaw);
            return new Ast(name, initial, ordered);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException(
                "Failed to reflect over StateMachine. On a recent JDK with strict module access, "
              + "you may need --add-opens=org.wpilib.command3/org.wpilib.command3=ALL-UNNAMED.",
                e);
        }
    }
 
    private static void populateWhenEdges(
            State s, StateNode node, Map<State, StateNode> nodeFor, ConditionLabeler labeler)
            throws ReflectiveOperationException {
        @SuppressWarnings("unchecked")
        List<Object> raw = (List<Object>) readField(State.class, "m_transitions").get(s);
        for (int i = 0; i < raw.size(); i++) {
            Object tx = raw.get(i);
            @SuppressWarnings("unchecked")
            Supplier<State> next = (Supplier<State>) readField(tx.getClass(), "m_nextSupplier").get(tx);
            BooleanSupplier cond = (BooleanSupplier) readField(tx.getClass(), "m_condition").get(tx);
            State target = safeGet(next);
            node.whenEdges.add(new Edge(
                Edge.Kind.WHEN,
                target == null ? null : nodeFor.get(target),
                labeler.label(cond, "t" + i),
                isUserSupplied(next),
                i));
        }
    }
 
    private static void populateWhenCompleteAndEdges(
            State s, StateNode node, Map<State, StateNode> nodeFor, ConditionLabeler labeler)
            throws ReflectiveOperationException {
        @SuppressWarnings("unchecked")
        List<Object> raw = (List<Object>) readField(State.class, "m_completions").get(s);
        for (int i = 0; i < raw.size(); i++) {
            Object cmp = raw.get(i);
            @SuppressWarnings("unchecked")
            Supplier<State> next = (Supplier<State>) readField(cmp.getClass(), "m_nextSupplier").get(cmp);
            BooleanSupplier cond = (BooleanSupplier) readField(cmp.getClass(), "m_condition").get(cmp);
            State target = safeGet(next);
            node.whenCompleteAndEdges.add(new Edge(
                Edge.Kind.WHEN_COMPLETE_AND,
                target == null ? null : nodeFor.get(target),
                labeler.label(cond, "c" + i),
                isUserSupplied(next),
                i));
        }
    }
 
    private static void populateDefaultCompletion(
            State s, StateNode node, Map<State, StateNode> nodeFor)
            throws ReflectiveOperationException {
        @SuppressWarnings("unchecked")
        Supplier<State> def = (Supplier<State>) readField(State.class, "m_defaultNextState").get(s);
        State target = safeGet(def);
        node.defaultCompletion = new Edge(
            Edge.Kind.WHEN_COMPLETE_DEFAULT,
            target == null ? null : nodeFor.get(target),
            null,
            isUserSupplied(def),
            0);
    }
 
    /** Calls {@code .get()} defensively; user-supplied dynamic suppliers may not be safe outside the scheduler. */
    private static State safeGet(Supplier<State> supplier) {
        try {
            return supplier.get();
        } catch (RuntimeException e) {
            return null;
        }
    }
 
    /**
     * Best-effort check for "did the user supply this {@link Supplier}, or did StateMachine
     * synthesize it internally?" Synthesized suppliers (from {@code .to(State)},
     * {@code .toExitStateMachine()}, etc.) all live inside the {@code org.wpilib.command3}
     * package; user-supplied dynamic suppliers do not.
     */
    private static boolean isUserSupplied(Supplier<State> supplier) {
        if (supplier == null) return false;
        return !supplier.getClass().getName().startsWith("org.wpilib.command3.");
    }
 
    // ---------------------------------------------------------------- Renderer
 
    /** Renders an AST to a Mermaid {@code stateDiagram-v2} source string. */
    public static String render(Ast ast) {
        StringBuilder sb = new StringBuilder(256);
        sb.append("---\n");
        sb.append("title: ").append(mermaidEscape(ast.name)).append('\n');
        sb.append("---\n");
        sb.append("stateDiagram-v2\n");
        sb.append("    direction LR\n\n");
 
        // Display labels: needed wherever the sanitized ID differs from the command name.
        for (StateNode s : ast.states) {
            if (!s.id.equals(s.label)) {
                sb.append("    ").append(s.id).append(" : ").append(mermaidEscape(s.label)).append('\n');
            }
        }
 
        if (ast.initial != null) {
            sb.append("    [*] --> ").append(ast.initial.id).append('\n');
        }
        sb.append('\n');
 
        for (StateNode s : ast.states) {
            for (Edge e : s.whenEdges) {
                emitEdge(sb, s, e, "when " + e.conditionLabel);
            }
            for (Edge e : s.whenCompleteAndEdges) {
                emitEdge(sb, s, e, "complete + " + e.conditionLabel);
            }
            // Always emit the default completion edge — it represents the path
            // taken when the state finishes without any whenCompleteAnd firing.
            emitEdge(sb, s, s.defaultCompletion, "complete");
        }
 
        return sb.toString();
    }
 
    private static void emitEdge(StringBuilder sb, StateNode from, Edge e, String label) {
        sb.append("    ")
          .append(from.id)
          .append(" --> ")
          .append(e.target == null ? "[*]" : e.target.id)
          .append(" : ")
          .append(e.dynamic ? "(dyn) " : "")
          .append(mermaidEscape(label))
          .append('\n');
    }
 
    // ---------------------------------------------------------------- Helpers
 
    private static Field readField(Class<?> owner, String name) throws NoSuchFieldException {
        Field f = owner.getDeclaredField(name);
        f.setAccessible(true);
        return f;
    }
 
    private static String uniqueId(String label, Map<String, Integer> counts) {
        String base = sanitize(label);
        if (base.isEmpty()) base = "state";
        int n = counts.getOrDefault(base, 0);
        counts.put(base, n + 1);
        return n == 0 ? base : base + "_" + n;
    }
 
    private static String sanitize(String s) {
        StringBuilder out = new StringBuilder(s.length());
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            if (Character.isLetterOrDigit(c)) {
                out.append(c);
            } else if (c == '_' || c == '-' || Character.isWhitespace(c)) {
                out.append('_');
            }
        }
        if (out.length() > 0 && Character.isDigit(out.charAt(0))) {
            out.insert(0, 's');
        }
        return out.toString();
    }
 
    private static String mermaidEscape(String s) {
        // Mermaid edge/state labels break on raw newlines and colons; replace them.
        return s.replace("\n", " ").replace(":", " -");
    }
 
    // ---------------------------------------------------------------- Condition labeling
 
    /** Strategy for turning a {@link BooleanSupplier} into a human-readable edge label. */
    @FunctionalInterface
    public interface ConditionLabeler {
        String label(BooleanSupplier condition, String fallback);
 
        /** Returns the fallback (e.g. {@code "t0"}, {@code "c1"}). Useful when conditions are anonymous lambdas. */
        static ConditionLabeler byOrder() {
            return (cond, fallback) -> fallback;
        }
 
        /** Uses the supplier's class simple name when it is meaningful; otherwise falls back. */
        static ConditionLabeler byClassName() {
            return (cond, fallback) -> {
                String s = cond.getClass().getSimpleName();
                if (s == null || s.isEmpty() || s.contains("Lambda") || s.contains("$$")) {
                    return fallback;
                }
                return s;
            };
        }
 
        /** Looks up labels from a registry (use {@link IdentityHashMap} to compare by reference). */
        static ConditionLabeler from(Map<BooleanSupplier, String> registry) {
            return (cond, fallback) -> registry.getOrDefault(cond, fallback);
        }
 
        /** Custom function; if it returns {@code null}, the fallback is used. */
        static ConditionLabeler of(Function<BooleanSupplier, String> fn) {
            return (cond, fallback) -> {
                String s = fn.apply(cond);
                return (s == null || s.isEmpty()) ? fallback : s;
            };
        }
    }

    public static void main(String[] args) {

        BooleanSupplier homeButton = () -> false;
        BooleanSupplier coralPickupButton = () -> false;
        BooleanSupplier hasCoral = () -> false;

        BooleanSupplier hasAlgae = () -> false;
        BooleanSupplier homeButtonAndHasAlgae = () -> homeButton.getAsBoolean() && hasAlgae.getAsBoolean();
        BooleanSupplier homeButtonAndHasNoAlgae = () -> homeButton.getAsBoolean() && !hasAlgae.getAsBoolean();

        BooleanSupplier l1ScoreButton = () -> false;
        BooleanSupplier l2ScoreButton = () -> false;
        BooleanSupplier l3ScoreButton = () -> false;
        BooleanSupplier l4ScoreButton = () -> false;

        BooleanSupplier l1ScoreButtonAndHasCoral = () -> l1ScoreButton.getAsBoolean() && hasCoral.getAsBoolean();
        BooleanSupplier l2ScoreButtonAndHasCoral = () -> l2ScoreButton.getAsBoolean() && hasCoral.getAsBoolean();
        BooleanSupplier l3ScoreButtonAndHasCoral = () -> l3ScoreButton.getAsBoolean() && hasCoral.getAsBoolean();
        BooleanSupplier l4ScoreButtonAndHasCoral = () -> l4ScoreButton.getAsBoolean() && hasCoral.getAsBoolean();

        BooleanSupplier l1ScoreButtonAndHasAlgae = () -> l1ScoreButton.getAsBoolean() && hasAlgae.getAsBoolean();
        BooleanSupplier l4ScoreButtonAndHasAlgae = () -> l4ScoreButton.getAsBoolean() && hasAlgae.getAsBoolean();

        BooleanSupplier algaeFloorPickupButton = () -> false;

        BooleanSupplier scoreButton = () -> false;


        var sm = new StateMachine("Arm and Elevator");

        State home = sm.addState(named("Home"));
        State coralPickup = sm.addState(named("Coral Pickup"));
        home.switchTo(coralPickup).when(coralPickupButton);
        coralPickup.switchTo(home).when(homeButton);

        State l1Score = sm.addState(named("L1 Score"));
        State l2Score = sm.addState(named("L2 Score"));
        State l3Score = sm.addState(named("L3 Score"));
        State l4Score = sm.addState(named("L4 Score"));
        State spitScoreCoral = sm.addState(named("Spit Score Coral"));
        State l2LowerScore = sm.addState(named("L2 Lower Score"));
        State l3LowerScore = sm.addState(named("L3 Lower Score"));
        State l4LowerScore = sm.addState(named("L4 Lower Score"));

        home.switchTo(l1Score).when(l1ScoreButtonAndHasCoral);
        home.switchTo(l2Score).when(l2ScoreButtonAndHasCoral);
        home.switchTo(l3Score).when(l3ScoreButtonAndHasCoral);
        home.switchTo(l4Score).when(l4ScoreButtonAndHasCoral);

        l1Score.switchTo(spitScoreCoral).when(scoreButton);
        l2Score.switchTo(spitScoreCoral).when(scoreButton);
        l3Score.switchTo(spitScoreCoral).when(scoreButton);
        l4Score.switchTo(spitScoreCoral).when(scoreButton);

        sm.switchFromAny(spitScoreCoral, l2LowerScore, l3LowerScore, l4LowerScore)
            .to(home)
            .when(homeButton);
        
        State algaeHome = sm.addState(named("Algae Home"));
        State algaeFloorPickup = sm.addState(named("Algae Floor Pickup"));
        State l2AlgaePickup = sm.addState(named("L2 Algae Pickup"));
        State l3AlgaePickup = sm.addState(named("L3 Algae Pickup"));
        State processorScorePosition = sm.addState(named("Processor Score Position"));
        State netScorePosition = sm.addState(named("Net Score Position"));

        home.switchTo(algaeHome).when(algaeFloorPickupButton);
        home.switchTo(l2AlgaePickup).when(l2ScoreButton);
        home.switchTo(l3AlgaePickup).when(l3ScoreButton);

        sm.switchFromAny(algaeFloorPickup, l2AlgaePickup, l3AlgaePickup)
            .to(algaeHome)
            .when(homeButtonAndHasAlgae);
        
        algaeHome.switchTo(netScorePosition).when(l4ScoreButtonAndHasAlgae);
        algaeHome.switchTo(processorScorePosition).when(l1ScoreButtonAndHasAlgae);

        netScorePosition.switchTo(algaeHome).when(homeButtonAndHasAlgae);
        netScorePosition.switchTo(processorScorePosition).when(l1ScoreButtonAndHasAlgae);
        netScorePosition.switchTo(home).when(homeButtonAndHasNoAlgae);

        processorScorePosition.switchTo(algaeHome).when(homeButtonAndHasAlgae);
        processorScorePosition.switchTo(netScorePosition).when(l4ScoreButtonAndHasAlgae);
        processorScorePosition.switchTo(home).when(homeButtonAndHasNoAlgae);


        Map<BooleanSupplier, String> labels = new IdentityHashMap<>();
        labels.put(homeButton, "homeButton");
        labels.put(coralPickupButton, "coralPickupButton");
        labels.put(hasCoral, "hasCoral");
        labels.put(hasAlgae, "hasAlgae");
        labels.put(homeButtonAndHasAlgae, "homeButton && hasAlgae");
        labels.put(homeButtonAndHasNoAlgae, "homeButton && !hasAlgae");
        labels.put(l1ScoreButton, "l1ScoreButton");
        labels.put(l2ScoreButton, "l2ScoreButton");
        labels.put(l3ScoreButton, "l3ScoreButton");
        labels.put(l4ScoreButton, "l4ScoreButton");
        labels.put(l1ScoreButtonAndHasCoral, "l1ScoreButton && hasCoral");
        labels.put(l2ScoreButtonAndHasCoral, "l2ScoreButton && hasCoral");
        labels.put(l3ScoreButtonAndHasCoral, "l3ScoreButton && hasCoral");
        labels.put(l4ScoreButtonAndHasCoral, "l4ScoreButton && hasCoral");
        labels.put(l1ScoreButtonAndHasAlgae, "l1ScoreButton && hasAlgae");
        labels.put(l4ScoreButtonAndHasAlgae, "l4ScoreButton && hasAlgae");
        labels.put(algaeFloorPickupButton, "algaeFloorPickupButton");
        labels.put(scoreButton, "scoreButton");
        
        System.out.println(Multiverse.toMermaid(
            sm, Multiverse.ConditionLabeler.from(labels)));
    }
 
    private static Command named(String name) {
        return new Command() {
            @Override public String name() { return name; }
            @Override public Set<Mechanism> requirements() { return Set.of(); }
            @Override public void run(Coroutine coroutine) { coroutine.yield(); }
        };
    }
}
