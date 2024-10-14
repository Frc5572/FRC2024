package frc.tools.modify;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import org.objectweb.asm.ClassVisitor;
import org.objectweb.asm.FieldVisitor;
import org.objectweb.asm.Label;
import org.objectweb.asm.MethodVisitor;
import org.objectweb.asm.Opcodes;

public class SimReplace extends ClassVisitor {

    private static final Map<String, String> replaceNames =
        Map.of("com/ctre/phoenix6/hardware/TalonFX", "frc/lib/sim/TalonFX");

    private static final Map<String, String> replaceDescriptors = new HashMap<>();

    static {
        for (var kv : replaceNames.entrySet()) {
            replaceDescriptors.put("L" + kv.getKey() + ";", "L" + kv.getValue() + ";");
        }
    }

    private static String replaceName(String s) {
        return replaceNames.getOrDefault(s, s);
    }

    private static String replaceDescriptor(String s) {
        for (var kv : replaceDescriptors.entrySet()) {
            s = s.replace(kv.getKey(), kv.getValue());
        }
        return s;
    }

    public SimReplace(ClassVisitor cv) {
        super(Opcodes.ASM9, cv);
    }

    @Override
    public MethodVisitor visitMethod(int access, String name, String descriptor, String signature,
        String[] exceptions) {
        return new SimReplaceMethod(super.visitMethod(access, name, replaceDescriptor(descriptor),
            signature == null ? null : replaceDescriptor(signature), exceptions == null ? null
                : Arrays.stream(exceptions).map(SimReplace::replaceName).toArray(String[]::new)));
    }

    @Override
    public FieldVisitor visitField(int access, String name, String descriptor, String signature,
        Object value) {
        return super.visitField(access, name, replaceDescriptor(descriptor),
            signature == null ? null : replaceDescriptor(signature), value);
    }

    private static class SimReplaceMethod extends MethodVisitor {

        public SimReplaceMethod(MethodVisitor mv) {
            super(Opcodes.ASM9, mv);
        }

        @Override
        public void visitFieldInsn(int opcode, String owner, String name, String descriptor) {
            super.visitFieldInsn(opcode, replaceName(owner), name, replaceDescriptor(descriptor));
        }

        @Override
        public void visitLocalVariable(String name, String descriptor, String signature,
            Label start, Label end, int index) {
            super.visitLocalVariable(name, replaceDescriptor(descriptor),
                signature == null ? null : replaceDescriptor(signature), start, end, index);
        }

        @Override
        public void visitTypeInsn(int opcode, String type) {
            super.visitTypeInsn(opcode, replaceName(type));
        }

        @Override
        public void visitTryCatchBlock(Label start, Label end, Label handler, String type) {
            super.visitTryCatchBlock(start, end, handler, replaceName(type));
        }

        @Override
        public void visitMethodInsn(int opcode, String owner, String name, String descriptor,
            boolean isInterface) {
            super.visitMethodInsn(opcode, replaceName(owner), name, replaceDescriptor(descriptor),
                isInterface);
        }

        @Override
        public void visitMultiANewArrayInsn(String descriptor, int numDimensions) {
            super.visitMultiANewArrayInsn(replaceDescriptor(descriptor), numDimensions);
        }

    }

}
