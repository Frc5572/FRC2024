package frc.tools;

import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.Collections;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;
import org.objectweb.asm.AnnotationVisitor;
import org.objectweb.asm.ClassReader;
import org.objectweb.asm.ClassVisitor;
import org.objectweb.asm.ClassWriter;
import org.objectweb.asm.MethodVisitor;
import org.objectweb.asm.Opcodes;

/** Jar file modifier that adds profiling commands */
public class AddProfiling {

    /** Entrypoint */
    public static void main(String[] args) throws IOException {
        FileSystem fs =
            FileSystems.newFileSystem(Paths.get("build/libs/FRC2024.jar"), Collections.emptyMap());
        FileOutputStream fout = new FileOutputStream("build/libs/FRC2024-mod.jar");
        ZipOutputStream zout = new ZipOutputStream(fout);
        for (Path p : fs.getRootDirectories()) {
            Files.walkFileTree(p, new FileWalker(zout));
        }
        zout.close();
        fs.close();
        Files.move(Paths.get("build/libs/FRC2024.jar"), Paths.get("build/libs/FRC2024-orig.jar"),
            StandardCopyOption.REPLACE_EXISTING);
        Files.move(Paths.get("build/libs/FRC2024-mod.jar"), Paths.get("build/libs/FRC2024.jar"),
            StandardCopyOption.REPLACE_EXISTING);
    }

    private static class FileWalker extends SimpleFileVisitor<Path> {

        private final ZipOutputStream out_fs;

        FileWalker(ZipOutputStream out_fs) {
            this.out_fs = out_fs;
        }

        @Override
        public FileVisitResult visitFile(Path filePath, BasicFileAttributes attrs)
            throws IOException {
            byte[] contents = Files.readAllBytes(filePath);
            ZipEntry ze = new ZipEntry(filePath.toString().substring(1));
            out_fs.putNextEntry(ze);
            if (!filePath.toString().endsWith(".class")) {
                out_fs.write(contents);
                out_fs.closeEntry();
                return FileVisitResult.CONTINUE;
            }
            if (filePath.toString().startsWith("/frc")) {
                // Code from this repo
                ClassReader reader = new ClassReader(contents);
                // Skip certain classes
                if (reader.getClassName().startsWith("frc/lib/profiling")
                    || reader.getClassName().startsWith("frc/lib/types")
                    || reader.getClassName().startsWith("frc/lib/math")
                    || reader.getClassName().startsWith("frc/lib/sim")
                    || reader.getClassName().equals("frc/robot/Main")
                    || reader.getClassName().equals("frc/robot/Robot")
                    || reader.getClassName().equals("frc/robot/RobotContainer")) {
                    out_fs.write(contents);
                    out_fs.closeEntry();
                    return FileVisitResult.CONTINUE;
                }
                RobotClassVisitor rcv = new RobotClassVisitor();
                reader.accept(rcv, 0);
                out_fs.write(rcv.toByteArray());
                out_fs.closeEntry();
                return FileVisitResult.CONTINUE;
            } else if (filePath.toString()
                .equals("/edu/wpi/first/wpilibj2/command/CommandScheduler.class")) {
                // We do some special logging inside of CommandScheduler
                ClassReader reader = new ClassReader(contents);
                CommandSchedulerClassVisitor cscv = new CommandSchedulerClassVisitor();
                reader.accept(cscv, 0);
                out_fs.write(cscv.toByteArray());
                out_fs.closeEntry();
                return FileVisitResult.CONTINUE;
            }
            out_fs.write(contents);
            out_fs.closeEntry();
            return FileVisitResult.CONTINUE;
        }
    }

    private static class RobotClassVisitor extends ClassVisitor {
        private final ClassWriter cw;
        private String className;

        protected RobotClassVisitor(ClassWriter cw) {
            super(Opcodes.ASM9, cw);
            this.cw = cw;
        }

        protected RobotClassVisitor() {
            this(new ClassWriter(ClassWriter.COMPUTE_FRAMES | ClassWriter.COMPUTE_MAXS));
        }

        public byte[] toByteArray() {
            return cw.toByteArray();
        }

        @Override
        public void visit(int version, int access, String name, String signature, String superName,
            String[] interfaces) {
            this.className = name;
            super.visit(version, access, name, signature, superName, interfaces);
        }

        @Override
        public MethodVisitor visitMethod(int access, String name, String descriptor,
            String signature, String[] exceptions) {
            if (name.equals("<clinit>") || name.equals("<init>") || name.equals("values")
                || name.equals("valueOf") || (access & Opcodes.ACC_SYNTHETIC) != 0) {
                return super.visitMethod(access, name, descriptor, signature, exceptions);
            }
            return new InstrumentMethodVisitor(
                super.visitMethod(access, name, descriptor, signature, exceptions), this.className,
                name);
        }
    }

    private static class CommandSchedulerClassVisitor extends ClassVisitor {
        private final ClassWriter cw;

        protected CommandSchedulerClassVisitor(ClassWriter cw) {
            super(Opcodes.ASM9, cw);
            this.cw = cw;
        }

        protected CommandSchedulerClassVisitor() {
            this(new ClassWriter(ClassWriter.COMPUTE_FRAMES | ClassWriter.COMPUTE_MAXS));
        }

        public byte[] toByteArray() {
            return cw.toByteArray();
        }
    }

    private static class InstrumentMethodVisitor extends MethodVisitor {

        private final String location;
        private boolean shouldSkip = false;

        protected InstrumentMethodVisitor(MethodVisitor mv, String className, String methodName) {
            super(Opcodes.ASM9, mv);
            this.location = className + ":" + methodName;
        }

        @Override
        public AnnotationVisitor visitAnnotation(String descriptor, boolean visible) {
            if (descriptor.equals("Lfrc/lib/profiling/SkipProfiling;")
                || descriptor.equals("Lorg/littletonrobotics/junction/AutoLogOutput;")) {
                shouldSkip = true;
            }
            return super.visitAnnotation(descriptor, visible);
        }

        private void push() {
            if (shouldSkip) {
                return;
            }
            this.mv.visitFieldInsn(Opcodes.GETSTATIC, "frc/robot/Robot", "profiler",
                "Lfrc/lib/profiling/Profiler;");
            this.mv.visitLdcInsn(location);
            this.mv.visitMethodInsn(Opcodes.INVOKEINTERFACE, "frc/lib/profiling/Profiler", "push",
                "(Ljava/lang/String;)V", true);
        }

        private void pop() {
            if (shouldSkip) {
                return;
            }
            this.mv.visitFieldInsn(Opcodes.GETSTATIC, "frc/robot/Robot", "profiler",
                "Lfrc/lib/profiling/Profiler;");
            this.mv.visitMethodInsn(Opcodes.INVOKEINTERFACE, "frc/lib/profiling/Profiler", "pop",
                "()V", true);
        }

        @Override
        public void visitCode() {
            super.visitCode();
            push();
        }

        @Override
        public void visitInsn(int opcode) {
            switch (opcode) {
                case Opcodes.ARETURN:
                case Opcodes.DRETURN:
                case Opcodes.FRETURN:
                case Opcodes.IRETURN:
                case Opcodes.LRETURN:
                case Opcodes.RETURN:
                    pop();
                    break;
                default:
                    break;

            }
            super.visitInsn(opcode);
        }

    }

}
