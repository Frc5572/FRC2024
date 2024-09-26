package frc.tools;

import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.StandardCopyOption;
import java.nio.file.StandardOpenOption;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.HashMap;
import org.objectweb.asm.ClassReader;
import org.objectweb.asm.ClassVisitor;
import org.objectweb.asm.Label;
import org.objectweb.asm.MethodVisitor;
import org.objectweb.asm.Opcodes;

public class VerifyProfiling {

    private static class CodeGraphNode {
        public final int labelNum;
        public final int lineNo;

        public CodeGraphNode(int labelNum, int lineNo) {
            this.labelNum = labelNum;
            this.lineNo = lineNo;
        }

        public ArrayList<CodeGraphNode> comesFrom = new ArrayList<>();
        public ArrayList<CodeGraphNode> goesTo = new ArrayList<>();
        public int pushes = 0;

        public String graphviz_name() {
            return "L" + labelNum;
        }

        public String graphviz_def() {
            String push = "(" + pushes + ")";
            if (lineNo == 0) {
                return graphviz_name() + " [shape=box,label=\"? " + push + "\"];";
            }
            return graphviz_name() + " [shape=box,label=\"Line " + lineNo + " " + push + "\"];";
        }

        public String graphviz_conn() {
            StringBuilder sb = new StringBuilder();
            for (CodeGraphNode to : goesTo) {
                sb.append(graphviz_name() + " -> " + to.graphviz_name() + ";\n");
            }
            return sb.toString();
        }
    }

    private static class EndGraphNode extends CodeGraphNode {
        private EndGraphNode() {
            super(-1, -1);
        }

        public static final EndGraphNode INSTANCE = new EndGraphNode();

        @Override
        public String graphviz_name() {
            return "return";
        }

        @Override
        public String graphviz_def() {
            return graphviz_name() + " [shape=box,label=\"Return\"];";
        }

        @Override
        public String graphviz_conn() {
            return "";
        }
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
            this.entry = new CodeGraphNode(numLabels++, 0);
            current = this.entry;
        }

        @Override
        public void visitLineNumber(int line, Label start) {
            recentLineNo = line;
        }

        @Override
        public void visitLabel(Label label) {
            CodeGraphNode newCurrent =
                graph.computeIfAbsent(label, k -> new CodeGraphNode(numLabels++, recentLineNo));
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
            CodeGraphNode to =
                graph.computeIfAbsent(label, k -> new CodeGraphNode(numLabels++, recentLineNo));
            to.comesFrom.add(this.current);
            this.current.goesTo.add(to);
        }

        @Override
        public void visitLookupSwitchInsn(Label dflt, int[] keys, Label[] labels) {
            this.reachedTerminal = true;
            for (int i = 0; i < labels.length; i++) {
                CodeGraphNode to = graph.computeIfAbsent(labels[i],
                    k -> new CodeGraphNode(numLabels++, recentLineNo));
                to.comesFrom.add(this.current);
                this.current.goesTo.add(to);
            }
            CodeGraphNode to =
                graph.computeIfAbsent(dflt, k -> new CodeGraphNode(numLabels++, recentLineNo));
            to.comesFrom.add(this.current);
            this.current.goesTo.add(to);
        }

        @Override
        public void visitTableSwitchInsn(int min, int max, Label dflt, Label... labels) {
            this.reachedTerminal = true;
            for (int i = 0; i < labels.length; i++) {
                CodeGraphNode to = graph.computeIfAbsent(labels[i],
                    k -> new CodeGraphNode(numLabels++, recentLineNo));
                to.comesFrom.add(this.current);
                this.current.goesTo.add(to);
            }
            CodeGraphNode to =
                graph.computeIfAbsent(dflt, k -> new CodeGraphNode(numLabels++, recentLineNo));
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
                System.out.println(path);
                StringBuilder sb = new StringBuilder();
                sb.append("digraph {\n");
                sb.append(entry.graphviz_def());
                sb.append('\n');
                sb.append(EndGraphNode.INSTANCE.graphviz_def());
                sb.append('\n');
                for (var entry : graph.values()) {
                    sb.append(entry.graphviz_def());
                    sb.append('\n');
                }
                sb.append(entry.graphviz_conn());
                for (var entry : graph.values()) {
                    sb.append(entry.graphviz_conn());
                }
                sb.append("}\n");
                Path p = Path.of("build/profiling/" + path.replace(".java:", "_").replace("(", "_")
                    .replace(")", "_").replace("/", "_").replace(";", "_") + ".dot");
                Path p2 = Path.of("build/profiling/" + path.replace(".java:", "_").replace("(", "_")
                    .replace(")", "_").replace("/", "_").replace(";", "_") + ".png");
                try {
                    Files.createDirectories(p.getParent());
                    Files.write(p, sb.toString().getBytes(), StandardOpenOption.CREATE);
                    var os = Runtime.getRuntime().exec("dot " + p.toString() + " -Tpng")
                        .getInputStream();
                    Files.copy(os, p2, StandardCopyOption.REPLACE_EXISTING);
                } catch (IOException e) {
                    e.printStackTrace();
                }
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
