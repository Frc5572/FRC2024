package frc.tools;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import frc.tools.modify.ClassTransformer;
import frc.tools.modify.SimReplace;

/** Class file modifier that adds profiling commands, replaces hardware in sim mode. */
public class ModifyClasses {

    /** Entrypoint */
    public static void main(String[] args) throws IOException {
        ClassTransformer transformer = new ClassTransformer();
        if (true) {
            transformer.add(SimReplace::new);
        }

        List<Path> result;
        try (Stream<Path> walk = Files.walk(Path.of("build/classes/java/main"))) {
            result = walk.filter(Files::isRegularFile)
                .filter(p -> p.getFileName().toString().endsWith(".class"))
                .filter(p -> !p.getFileName().toString().endsWith("Robot$1.class"))
                .collect(Collectors.toList());
        }

        for (Path p : result) {
            byte[] input = Files.readAllBytes(p);
            byte[] output = transformer.transform(input);
            Files.write(p, output, StandardOpenOption.WRITE, StandardOpenOption.TRUNCATE_EXISTING);
        }

    }

}
