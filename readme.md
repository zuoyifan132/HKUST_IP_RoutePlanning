# <center>Route Planning with DQN<center/>

This repository contains a Deep Q-Network (DQN) implementation for solving Multi-Agent Path Finding (MAPF) problems. 

## Requirements

- Python 3.6 or later
- PyTorch
- NumPy

## Usage

To train the DQN on a specific MAPF instance, run the following command:

```
python train.py --instance <file_name>
```

where `<file_name>` is the name of the MAPF instance file. The instance file should be in MAPF format.

## MAPF Instance Format

The MAPF instance file should be in the following format:

```
<map_height> <map_width>
<start_x_1> <start_y_1> <goal_x_1> <goal_y_1>
...
<start_x_n> <start_y_n> <goal_x_n> <goal_y_n>
<map_row_1>
...
<map_row_h>
```

where:
- `<map_height>` and `<map_width>` are the height and width of the map, respectively.
- `<start_x_i>` and `<start_y_i>` are the x and y coordinates of the starting position of agent `i`.
- `<goal_x_i>` and `<goal_y_i>` are the x and y coordinates of the goal position of agent `i`.
- `<map_row_j>` is the j-th row of the map, where `.` represents a free grid, `#` represents a blocked grid, and `1` to `n` represent the positions of agents.

## Acknowledgements

This implementation is based on the DQN algorithm and architecture proposed by [Mnih et al. (2015)](https://www.nature.com/articles/nature14236). The MAPF instances used in the experiments are from the [Moving AI Lab repository](https://www.movingai.com/benchmarks/mapf.html).

## License

This project is licensed under the MIT License - see the LICENSE file for details.