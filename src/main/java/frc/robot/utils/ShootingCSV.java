package frc.robot.utils;

import com.opencsv.CSVReader;
import com.opencsv.exceptions.CsvValidationException;
import edu.wpi.first.math.Pair;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import lib.math.interpolation.InterpolatingDoubleMap;

public class ShootingCSV {

    public static InterpolatingDoubleMap parse(String file) {
        InterpolatingDoubleMap map = new InterpolatingDoubleMap(100);
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
                .map(
                        (pair) ->
                                new Pair<>(
                                        Double.parseDouble(pair.getFirst()),
                                        Double.parseDouble(pair.getSecond())))
                .forEach((pair) -> map.put(pair.getFirst(), pair.getSecond()));
        return map;
    }
}
