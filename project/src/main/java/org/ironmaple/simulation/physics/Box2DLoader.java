package org.ironmaple.simulation.physics;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

/**
 *
 *
 * <h2>Native Library Loader for Box2D JNI.</h2>
 *
 * <p>Handles loading the native Box2D library for the current platform.
 */
public final class Box2DLoader {
    private static boolean loaded = false;
    private static final String LIBRARY_NAME = "box2djni";

    private Box2DLoader() {}

    /**
     *
     *
     * <h2>Load the Box2D native library.</h2>
     *
     * <p>This method loads the native library for the current platform. It first tries to load from the system path,
     * then falls back to extracting from the JAR.
     */
    public static synchronized void load() {
        if (loaded) return;

        try {
            // First try loading from system path
            System.loadLibrary(LIBRARY_NAME);
            loaded = true;
            return;
        } catch (UnsatisfiedLinkError e) {
            // Fall through to extract from JAR
        }

        // Determine platform-specific library name
        String os = System.getProperty("os.name").toLowerCase();
        String arch = System.getProperty("os.arch").toLowerCase();
        String libName;
        String resourcePath;

        if (os.contains("win")) {
            libName = LIBRARY_NAME + ".dll";
            resourcePath = "/native/windows-" + (arch.contains("64") ? "x86-64" : "x86") + "/" + libName;
        } else if (os.contains("mac")) {
            libName = "lib" + LIBRARY_NAME + ".dylib";
            resourcePath = "/native/osx-" + (arch.contains("aarch64") ? "arm64" : "x86-64") + "/" + libName;
        } else {
            libName = "lib" + LIBRARY_NAME + ".so";
            if (arch.contains("arm") && arch.contains("64")) {
                resourcePath = "/native/linux-arm64/" + libName;
            } else if (arch.contains("arm")) {
                resourcePath = "/native/linux-arm32/" + libName;
            } else {
                resourcePath = "/native/linux-x86-64/" + libName;
            }
        }

        try {
            // Extract from JAR to temp file
            InputStream in = Box2DLoader.class.getResourceAsStream(resourcePath);
            if (in == null) {
                throw new UnsatisfiedLinkError("Native library not found in JAR: " + resourcePath);
            }

            File tempFile = File.createTempFile(LIBRARY_NAME, libName.substring(libName.lastIndexOf('.')));
            tempFile.deleteOnExit();

            try (FileOutputStream out = new FileOutputStream(tempFile)) {
                byte[] buffer = new byte[8192];
                int read;
                while ((read = in.read(buffer)) != -1) {
                    out.write(buffer, 0, read);
                }
            }
            in.close();

            System.load(tempFile.getAbsolutePath());
            loaded = true;
        } catch (IOException e) {
            throw new UnsatisfiedLinkError("Failed to extract native library: " + e.getMessage());
        }
    }
}
