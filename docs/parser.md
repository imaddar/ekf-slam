# EuRoC Parser Notes

## Current Approach

The parser currently favors simple, testable parsing steps:

1. Read external data into parser-specific raw types.
2. Validate the raw shape.
3. Convert the raw values into domain structs used by the rest of the program.

For YAML calibration files, this means deserializing into raw structs that match the
EuRoC file format, then converting into `CameraCalibration` or `ImuCalibration`.

For CSV files, the current parser reads the file contents and returns vectors of
domain structs, such as `Vec<ImuMeasurement>` or `Vec<GroundTruthState>`.

## Future Considerations

### Streaming CSV Parsing

The current CSV implementation is acceptable for early development and
EuRoC-scale files, but it reads the full CSV into memory before parsing. Once the
CSV behavior is stable, consider switching file-level parsing to `BufReader`.

Reasons to revisit this:

- Avoid holding both the raw CSV string and parsed measurements in memory.
- Start parsing before the full file has been read.
- Fail earlier when a malformed row appears near the beginning of a file.
- Better support larger datasets, longer recordings, or multiple sequences.

This does not require changing the domain structs. It only changes how the parser
feeds lines into the row-level parsing functions.

### Iterator-Based Sensor Playback

Longer term, the EKF may not need every parsed sensor record in memory at once.
Instead, it may be cleaner to expose a time-ordered stream of sensor events:

```text
IMU measurement
IMU measurement
camera frame
IMU measurement
ground-truth state
```

That would fit an online EKF pipeline better than loading every file into a set of
vectors first. A `BufReader`-based parser is a natural stepping stone toward this
kind of event-stream API.
