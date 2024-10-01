package frc.tools;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.Collections;
import org.objectweb.asm.ClassReader;
import org.objectweb.asm.ClassWriter;

public class AddProfiling {



    private static class FileWalker extends SimpleFileVisitor<Path> {

        @Override
        public FileVisitResult visitFile(Path filePath, BasicFileAttributes attrs)
            throws IOException {
            if (!filePath.toString().endsWith(".class")) {
                return FileVisitResult.CONTINUE;
            }
            if (filePath.toString().startsWith("/frc")) {
                // Code from this repo
                byte[] contents = Files.readAllBytes(filePath);
                ClassReader reader = new ClassReader(contents);
                // Skip certain classes
                if (reader.getClassName().startsWith("frc/lib/profiling")
                    || reader.getClassName().equals("frc/robot/Main")
                    || reader.getClassName().equals("frc/robot/Robot")
                    || reader.getClassName().equals("frc/robot/RobotContainer")) {
                    return FileVisitResult.SKIP_SIBLINGS;
                }
                System.out.println(reader.getClassName());
                ClassWriter writer =
                    new ClassWriter(ClassWriter.COMPUTE_FRAMES | ClassWriter.COMPUTE_MAXS);
                reader.accept(writer, 0);
                Files.write(filePath, writer.toByteArray());
                return FileVisitResult.CONTINUE;
            } else if (filePath.toString()
                .equals("/edu/wpi/first/wpilibj2/command/CommandScheduler.class")) {
                // We do some special logging inside of CommandScheduler
                byte[] contents = Files.readAllBytes(filePath);
                ClassReader reader = new ClassReader(contents);
                System.out.println(reader.getClassName());
                ClassWriter writer =
                    new ClassWriter(ClassWriter.COMPUTE_FRAMES | ClassWriter.COMPUTE_MAXS);
                reader.accept(writer, 0);
                Files.write(filePath, writer.toByteArray());
                return FileVisitResult.CONTINUE;
            }
            return FileVisitResult.CONTINUE;
        }

    }

    public static void main(String[] args) throws IOException {
        FileSystem fs =
            FileSystems.newFileSystem(Paths.get("build/libs/FRC2024.jar"), Collections.emptyMap());
        for (Path p : fs.getRootDirectories()) {
            Files.walkFileTree(p, new FileWalker());
        }
    }

}
