package frc.tools;

import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.concurrent.atomic.AtomicInteger;
import org.objectweb.asm.ClassReader;
import org.objectweb.asm.ClassVisitor;
import org.objectweb.asm.Label;
import org.objectweb.asm.MethodVisitor;
import org.objectweb.asm.Opcodes;
import frc.lib.types.Tuples.Tuple2;

public class VerifyProfiling {

    private static record GraphWalkHistory(int[] lineNoHistory, int currentStackSize,
        CodeGraphNode currentEntry) {
        @Override
        public final boolean equals(Object arg0) {
            if (arg0 instanceof GraphWalkHistory other) {
                if (other.currentEntry.lineNo == 0 || this.currentEntry.lineNo == 0) {
                    return false;
                }
                return currentStackSize == other.currentStackSize
                    && currentEntry.lineNo == other.currentEntry.lineNo;
            }
            return false;
        }

        @Override
        public final int hashCode() {
            int hash = 17;
            hash = hash * 31 + this.currentStackSize;
            hash = hash * 31 + this.currentEntry.lineNo;
            return hash;
        }

        @Override
        public final String toString() {
            StringBuilder sb = new StringBuilder();
            sb.append("[currentStackSize=");
            sb.append(currentStackSize);
            sb.append(",lineNo=");
            sb.append(currentEntry.lineNo);
            sb.append("]");
            return sb.toString();
        }
    }

    private static boolean verifyGraph(CodeGraphNode entry, int maxStackSize, int maxHistory) {
        GraphWalkHistory[] currentPaths =
            new GraphWalkHistory[] {new GraphWalkHistory(new int[] {}, 0, entry)};
        for (int _i = 0; _i < maxHistory; _i++) {
            ArrayList<GraphWalkHistory> newPaths = new ArrayList<>();
            System.out.print("[");
            for (int i = 0; i < currentPaths.length; i++) {
                if (i != 0) {
                    System.out.print(", ");
                }
                System.out.print(currentPaths[i]);
            }
            System.out.println("]");
            for (int i = 0; i < currentPaths.length; i++) {
                int newStackSize =
                    currentPaths[i].currentStackSize + currentPaths[i].currentEntry.pushes;
                int[] newHistory = new int[currentPaths[i].lineNoHistory.length + 1];
                System.arraycopy(currentPaths[i].lineNoHistory, 0, newHistory, 0,
                    currentPaths[i].lineNoHistory.length);
                for (var next : currentPaths[i].currentEntry.goesTo) {
                    var nextEntry = new GraphWalkHistory(newHistory, newStackSize, next);
                    if (!newPaths.contains(nextEntry)) {
                        newPaths.add(nextEntry);
                    } else {
                        System.out
                            .println("skipping adding " + nextEntry + " since already in new");
                    }
                }
            }
            currentPaths = newPaths.toArray(GraphWalkHistory[]::new);
        }
        return true;
    }

    private static class CodeGraphNode {
        public final int lineNo;

        public CodeGraphNode(int lineNo) {
            this.lineNo = lineNo;
        }

        public ArrayList<CodeGraphNode> comesFrom = new ArrayList<>();
        public ArrayList<CodeGraphNode> goesTo = new ArrayList<>();
        public int pushes = 0;
    }

    private static class EndGraphNode extends CodeGraphNode {
        private EndGraphNode() {
            super(-1);
        }

        public static final EndGraphNode INSTANCE = new EndGraphNode();
    }

    private static class MethodWalker extends MethodVisitor {

        private final String path;
        private int recentLineNo = 0;
        private CodeGraphNode current;
        private HashMap<Label, CodeGraphNode> graph = new HashMap<>();
        private final CodeGraphNode entry;
        private boolean reachedTerminal = false;
        private boolean seesPush = false;

        private int numLabels = 0;

        public MethodWalker(String path) {
            super(Opcodes.ASM9);
            this.path = path;
            this.entry = new CodeGraphNode(0);
            current = this.entry;
        }

        @Override
        public void visitLineNumber(int line, Label start) {
            recentLineNo = line;
        }

        @Override
        public void visitLabel(Label label) {
            CodeGraphNode newCurrent =
                graph.computeIfAbsent(label, k -> new CodeGraphNode(recentLineNo));
            if (!reachedTerminal) {
                current.goesTo.add(newCurrent);
                newCurrent.comesFrom.add(current);
            } else {
                reachedTerminal = false;
            }
            current = newCurrent;
        }

        @Override
        public void visitInsn(int opcode) {
            switch (opcode) {
                case Opcodes.RETURN:
                case Opcodes.ARETURN:
                case Opcodes.DRETURN:
                case Opcodes.FRETURN:
                case Opcodes.IRETURN:
                case Opcodes.LRETURN:
                    this.reachedTerminal = true;
                    this.current.goesTo.add(EndGraphNode.INSTANCE);
                    break;
                default:
                    break;
            }
        }

        @Override
        public void visitJumpInsn(int opcode, Label label) {
            switch (opcode) {
                case Opcodes.GOTO:
                case Opcodes.JSR:
                    this.reachedTerminal = true;
                    break;
                default:
                    break;
            }
            CodeGraphNode to = graph.computeIfAbsent(label, k -> new CodeGraphNode(recentLineNo));
            to.comesFrom.add(this.current);
            this.current.goesTo.add(to);
        }

        @Override
        public void visitLookupSwitchInsn(Label dflt, int[] keys, Label[] labels) {
            this.reachedTerminal = true;
            for (int i = 0; i < labels.length; i++) {
                CodeGraphNode to =
                    graph.computeIfAbsent(labels[i], k -> new CodeGraphNode(recentLineNo));
                to.comesFrom.add(this.current);
                this.current.goesTo.add(to);
            }
            CodeGraphNode to = graph.computeIfAbsent(dflt, k -> new CodeGraphNode(recentLineNo));
            to.comesFrom.add(this.current);
            this.current.goesTo.add(to);
        }

        @Override
        public void visitTableSwitchInsn(int min, int max, Label dflt, Label... labels) {
            this.reachedTerminal = true;
            for (int i = 0; i < labels.length; i++) {
                CodeGraphNode to =
                    graph.computeIfAbsent(labels[i], k -> new CodeGraphNode(recentLineNo));
                to.comesFrom.add(this.current);
                this.current.goesTo.add(to);
            }
            CodeGraphNode to = graph.computeIfAbsent(dflt, k -> new CodeGraphNode(recentLineNo));
            to.comesFrom.add(this.current);
            this.current.goesTo.add(to);
        }

        @Override
        public void visitMethodInsn(int opcode, String owner, String name, String descriptor,
            boolean isInterface) {
            if (owner.equals("frc/lib/profiling/Profiler")
                || owner.equals("frc/lib/profiling/LoggingProfiler")
                || owner.equals("frc/lib/profiling/EmptyProfiler")
                || owner.equals("frc/lib/profiling/ValidatingProfiler")) {
                if (name.equals("push")) {
                    current.pushes++;
                    seesPush = true;
                } else if (name.equals("pop")) {
                    current.pushes--;
                    seesPush = true;
                }
            }
        }

        @Override
        public void visitEnd() {
            if (!current.goesTo.contains(EndGraphNode.INSTANCE)) {
                current.goesTo.add(EndGraphNode.INSTANCE);
            }
            if (seesPush) {
                HashMap<Integer, Tuple2<HashSet<Integer>, AtomicInteger>> consolidated =
                    new HashMap<>();
                {
                    Tuple2<HashSet<Integer>, AtomicInteger> ce = consolidated.computeIfAbsent(
                        entry.lineNo, _k -> new Tuple2<>(new HashSet<>(), new AtomicInteger(0)));
                    ce._1().addAndGet(entry.pushes);
                    for (var e : entry.goesTo) {
                        if(e.lineNo != entry.lineNo) {
                            ce._0().add(e.lineNo);
                        }
                    }
                }
                for (var entry : graph.values()) {
                    Tuple2<HashSet<Integer>, AtomicInteger> ce = consolidated.computeIfAbsent(
                        entry.lineNo, _k -> new Tuple2<>(new HashSet<>(), new AtomicInteger(0)));
                    ce._1().addAndGet(entry.pushes);
                    for (var e : entry.goesTo) {
                        if(e.lineNo != entry.lineNo) {
                            ce._0().add(e.lineNo);
                        }
                    }
                }
                System.out.println(path);
                System.out.println("[");
                for (var entry : consolidated.entrySet()) {
                    System.out.println("    ");
                    System.out.print(entry.getKey());
                    System.out.println(":(");
                    System.out.print("        [");
                    var next = entry.getValue()._0().toArray(Integer[]::new);
                    for (int i = 0; i < next.length; i++) {
                        System.out.print(next[i]);
                        System.out.print(",");
                    }
                    System.out.println("],");
                    System.out.print("        ");
                    System.out.println(entry.getValue()._1().get());
                    System.out.println("    )");
                }
                System.out.println("]");
                // verifyGraph(consolidated.get(0), 10, 200);
            }
        }

    }

    private static class ClassWalker extends ClassVisitor {

        private String path;

        public ClassWalker(String path) {
            super(Opcodes.ASM9);
            this.path = path;
        }

        @Override
        public void visitSource(String source, String debug) {
            if (source != null) {
                path = source;
            }
        }

        @Override
        public MethodVisitor visitMethod(int access, String name, String descriptor,
            String signature, String[] exceptions) {
            return new MethodWalker(this.path + ":" + name + descriptor);
        }

    }

    private static class FileWalker extends SimpleFileVisitor<Path> {

        @Override
        public FileVisitResult visitFile(Path filePath, BasicFileAttributes attrs)
            throws IOException {
            byte[] contents = Files.readAllBytes(filePath);
            ClassReader reader = new ClassReader(contents);
            if (reader.getClassName().startsWith("frc/lib/profiling")) {
                return FileVisitResult.SKIP_SIBLINGS;
            }
            reader.accept(new ClassWalker(filePath.toString()), 0);
            return FileVisitResult.CONTINUE;
        }

    }

    public static void main(String[] args) throws IOException {
        Path startingDir = Path.of("build/classes/java/main");
        Files.walkFileTree(startingDir, new FileWalker());
    }

}
