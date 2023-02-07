# SIMULATED ANNEALING ALGORITHM

This repository contains the code for a Simulated Annealing algorithm that finds the maximum value of a function f. The function f is defined as f = |14 * one(v) - 190|, where v is a 50-bit binary input variable, and one(v) is the number of ones in v. The algorithm is reset 200 times to find the global maximum value of the function.

## Requirements:

* Python 3.x
* Jupyter Notebook

## Running the code:

To run the code, simply run the following command in your terminal or command prompt: 

* Clone the repository to your local machine using the command ‘git clone https://github.com/padam56/PhD_Coursework_UNO/tree/main/Machine%20Learning%20II/Assignments/Assignment-I/Simulated%20Annealing'
* Open the Jupyter Notebook file named Simulated_Annealing.ipynb
* Run the code in the Jupyter Notebook by clicking on "Cell" in the top menu and then selecting "Run All"

## Output:

The output.txt file will contain a list of 200 comma-separated values, where each value represents the maximum value of the function f calculated by the Simulated Annealing algorithm in each iteration. The function f is defined as f = |14 * one(v) - 190|, where one(v) is the number of 1's in the binary input variable v, which is a randomly generated 50-bit binary number.

The Simulated Annealing algorithm will perform 200 iterations, and in each iteration, it will generate a random 50-bit binary number, calculate the value of f for that number, and update the maximum value of f if the newly calculated value is greater than the current maximum. The maximum value of f found in each iteration will be recorded and written to the output.txt file, separated by a comma.

The output.txt file will provide the result of the Simulated Annealing algorithm applied to the function f and can be used to analyze the performance and efficiency of the algorithm.


## Time and Space Complexities:

The time complexity of this Simulated Annealing algorithm is O(MAX * t), where MAX is the number of times the algorithm is reset, and t is the number of iterations the algorithm runs in each reset. This is because the algorithm takes a constant amount of time to evaluate the value of the function f and to update the current state and current maximum value.

The space complexity of this algorithm is O(1), as it only requires a constant amount of memory to store the current state, the current maximum value, and some variables to control the algorithm.

It is important to note that the time complexity can be affected by the choice of the cooling rate, as well as the maximum number of iterations allowed in each temperature.