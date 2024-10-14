package frc.tools.modify;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import org.objectweb.asm.ClassReader;
import org.objectweb.asm.ClassVisitor;
import org.objectweb.asm.ClassWriter;

public class ClassTransformer {

    private List<Function<ClassVisitor, ClassVisitor>> transforms = new ArrayList<>();

    public ClassTransformer() {}

    public void add(Function<ClassVisitor, ClassVisitor> func) {
        this.transforms.add(func);
    }

    public byte[] transform(byte[] input) {
        ClassReader reader = new ClassReader(input);
        return transform(reader);
    }

    public byte[] transform(ClassReader reader) {
        ClassWriter writer = new ClassWriter(ClassWriter.COMPUTE_FRAMES | ClassWriter.COMPUTE_MAXS);
        ClassVisitor cv = writer;
        for (var transform : transforms) {
            cv = transform.apply(cv);
        }
        reader.accept(cv, 0);
        return writer.toByteArray();
    }

}
