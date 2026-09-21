# Planning

## Compilaton

> Note: If you want clangd to provide good autocorrect do: `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1` and `cp build/compile_commands.json .`

Build with: `colcon build`.

## Usage

Planning takes in map data and outputs two objects: 
1. TargetPath. A list of waypoints relative to the car.
2. SpeedProfile. A list of speeds that control should follow.

These are then ingested by control.

## Debugging

See [here](https://www.notion.so/Debugging-C-With-GDB-2b86a4110fc680388d27d07f0c63fa4f?source=copy_link) for documentation.