package frc.robot.utils;

import com.opencsv.CSVReader;
import com.opencsv.exceptions.CsvValidationException;
import edu.wpi.first.math.Pair;
import lib.math.interpolation.InterpolatingDoubleMap;

import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class ShootingCSV {

    public static void parse(String file, InterpolatingDoubleMap map) {
        List<Pair<String, String>> records = new ArrayList<>();
        try (CSVReader reader = new CSVReader(new FileReader(file))) {
            String[] values;
            while ((values = reader.readNext()) != null) {
                records.add(new Pair<>(values[0], values[1]));
            }
        } catch (CsvValidationException | IOException e) {
            throw new RuntimeException(e);
        }

        records.stream()
                .map((pair) -> new Pair<>(Double.parseDouble(pair.getFirst()), Double.parseDouble(pair.getSecond())))
                .forEach((pair) -> map.put(pair.getFirst(), pair.getSecond()));
    }
}
