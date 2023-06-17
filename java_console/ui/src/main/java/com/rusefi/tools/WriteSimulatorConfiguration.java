package com.rusefi.tools;

import com.opensr5.ConfigurationImage;
import com.opensr5.ini.IniFileModel;
import com.rusefi.binaryprotocol.MsqFactory;
import com.rusefi.config.generated.Fields;
import com.rusefi.tune.xml.Msq;

import javax.xml.bind.JAXBException;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Objects;

public class WriteSimulatorConfiguration {
    // f407-discovery is historically the most inclusive .ini file
    private static final String INI_FILE_FOR_SIMULATOR = "../firmware/tunerstudio/generated/rusefi_f407-discovery.ini";
    private static final String FOLDER = "generated";

    public static void main(String[] args) throws IOException, InterruptedException, JAXBException {
        try {
            writeTune();
        } finally {
            // No way to set Process.exec to be a daemon, we need explicit exit
            System.exit(0);
        }
    }

    private static void writeTune() throws JAXBException, IOException {
        String s = "generated/simulator_tune_image.bin";
        byte[] fileContent = Files.readAllBytes(new File(s).toPath());
        System.out.println("Got " + fileContent.length + " from " + s + " while expecting " + Fields.TOTAL_CONFIG_SIZE);
        if (fileContent.length != Fields.TOTAL_CONFIG_SIZE)
            throw new IllegalStateException("Unexpected image size " + fileContent.length);
        ConfigurationImage configuration = new ConfigurationImage(fileContent);
        System.out.println("Got " + Objects.requireNonNull(configuration, "configuration"));
        IniFileModel ini = new IniFileModel().readIniFile(INI_FILE_FOR_SIMULATOR);
        if (ini == null)
            throw new IllegalStateException("Not found " + INI_FILE_FOR_SIMULATOR);
        Msq m = MsqFactory.valueOf(configuration, ini);
        m.writeXmlFile(FOLDER + File.separator + "simulator_tune.xml");
    }
}
