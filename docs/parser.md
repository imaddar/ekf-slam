# EuRoC Parser Notes

## Current Approach

The parser favors simple, testable parsing steps:

1. Read external data into parser-specific raw types.
2. Validate the raw shape.
3. Convert the raw values into domain structs used by the rest of the program.

For YAML calibration files, this means parsing into raw key/value and transform
structs that match the EuRoC file format, then converting into
`CameraCalibration` or `ImuCalibration`. Both YAML and CSV parsing are
hand-written rather than backed by a third-party library, which is what lets
errors name the field, the line number, and the expected vs. found shape.

For CSV files, the parser reads the file contents and returns vectors of domain
structs — `std::vector<ImuMeasurement>`, `std::vector<GroundTruthState>`,
`std::vector<StereoPair>`. Every fallible step returns
`ParseResult<T> = std::expected<T, std::string>`; nothing throws.

Only `parse_dataset(...)` is public. The per-file functions live in
`parser_detail` inside `parser_csv.cpp` and `parser_yaml.cpp`.

## Future Considerations

### Streaming CSV Parsing

The current implementation is acceptable for early development and EuRoC-scale
files, but it reads each full CSV into a `std::string` before parsing. MH_01
alone yields 36,820 IMU samples, 3,682 stereo pairs, and 36,382 ground-truth
states, all materialized before `parse_dataset(...)` returns. Once the CSV
behavior is stable, consider parsing line by line straight off the `std::ifstream`
with `std::getline`.

Reasons to revisit this:

- Avoid holding both the raw CSV string and the parsed measurements in memory.
- Start parsing before the full file has been read.
- Fail earlier when a malformed row appears near the beginning of a file.
- Better support larger datasets, longer recordings, or multiple sequences.

This does not require changing the domain structs. It only changes how the parser
feeds lines into the row-level parsing functions.

### Iterator-Based Sensor Playback

Longer term, the filter does not need every parsed sensor record in memory at
once. A time-ordered stream of sensor events would fit better:

```text
IMU measurement
IMU measurement
camera frame
IMU measurement
ground-truth state
```

That matches how the propagate/update loop actually consumes data, and it is the
shape the Phase 2 real-time pipeline needs, where records arrive from ROS 2
subscriptions rather than from files. Line-by-line CSV parsing is a natural
stepping stone toward this kind of event-stream API.

The reader types this would introduce are sketched in [roadmap.md](../roadmap.md).
