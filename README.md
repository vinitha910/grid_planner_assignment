# Grid Planner Assignment

In this assignment you will need to implement Dijkstra's algorithm and the graph representation it will use. The header files `graph.h` and `dijkstra.h` contain the information you will need to fill in the functions in `graph.cpp` and `dijkstra.cpp` respectively.

You will need to install `libjsoncpp` to run/test your solution and `numpy` and `matplotlib` for visualizing your solution. Run the following commands to install these dependencies:

```bash
$ sudo apt-get install libjsoncpp-dev libjsoncpp1
$ sudo pip install numpy matplotlib
```

To build your code run `make` and to test your code run `./test` (make sure you create a directory called `student_solutions`, the planner will output the solutions from running your code in that directory)

You will only need to modify `graph.h`, `graph.cpp` and `dijkstra.cpp`. *Do not* modify the other files.

To visualize your solution run the following command:

```bash
python visualize.py environments/<ENV_TXT_FILE> student_solutions/<SOLUTION_FILE>
```

Look at `tests.json` to find the environment file for the appropiate test.

To visualize the actual solution run the following command:

```bash
python visualize.py environments/<ENV_TXT_FILE> solutions/<SOLUTION_FILE>
```
