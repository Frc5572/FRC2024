package frc.robot;

import static org.lwjgl.system.MemoryStack.stackPush;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.ByteBuffer;
import org.littletonrobotics.junction.LogFileUtil;
import org.lwjgl.PointerBuffer;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.util.nfd.NFDFilterItem;
import org.lwjgl.util.nfd.NativeFileDialog;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

/**  */
public class Replay {

    /**  */
    public static void main(String[] argv) {
        File prevPath = new File(Filesystem.getLaunchDirectory(), "prev-log.txt");
        NativeFileDialog.NFD_Init();
        String logPath;
        try (MemoryStack stack = stackPush()) {
            NFDFilterItem.Buffer filters = NFDFilterItem.malloc(1);
            filters.get(0).name(stack.UTF8("WPILog Files")).spec(stack.UTF8("wpilog"));

            ByteBuffer defaultPath = null;
            if (prevPath.exists()) {
                String prev = new File(readToString(prevPath).trim()).getParent();
                defaultPath = stack.UTF8(prev);
            }

            PointerBuffer pp = stack.mallocPointer(1);
            int res = NativeFileDialog.NFD_OpenDialog(pp, filters, defaultPath);
            if (res != NativeFileDialog.NFD_OKAY) {
                return;
            }
            logPath = pp.getStringUTF8();
        }
        NativeFileDialog.NFD_Quit();
        try {
            try (PrintWriter out = new PrintWriter(prevPath)) {
                out.print(logPath);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        Robot.setReplayFile(logPath);
        RobotBase.startRobot(Robot::new);
    }

    private static String readToString(File f) {
        BufferedReader reader;
        try {
            reader = new BufferedReader(new FileReader(f));
            StringBuilder stringBuilder = new StringBuilder();
            String line = null;
            String ls = System.getProperty("line.separator");
            while ((line = reader.readLine()) != null) {
                stringBuilder.append(line);
                stringBuilder.append(ls);
            }
            // delete the last new line separator
            stringBuilder.deleteCharAt(stringBuilder.length() - 1);
            reader.close();
            return stringBuilder.toString();
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

}
