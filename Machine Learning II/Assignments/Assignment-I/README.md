# HILL CLIMBING ALGORITHM

This code implements a Hill-climbing algorithm to find the maximum value of a function f, where f = |13 * one(v) - 170|. The input variable ‘v’ is a binary variable of 40 bits, and the one function counts the number of 1's in v. The algorithm resets 100 times and finds the maximum value for each reset, printing the found maximum values to an "Output.txt" file, separated by commas.

## Requirements:

* Python 3.x
* Jupyter Notebook

## Running the code:

To run the code, simply run the following command in your terminal or command prompt: 

* Clone the repository to your local machine using the command ‘git clone https://github.com/padam56/machinelearningII/Assignment-I.git.
* Open the Jupyter Notebook file named hill_climbing.ipynb
* Run the code in the Jupyter Notebook by clicking on "Cell" in the top menu and then selecting "Run All"

## Output:

The output of the Hill-Climbing algorithm will be a sequence of 100 values, separated by commas, written to the "Output.txt" file. These values represent the maximum value of the function "f" calculated by the algorithm in each of its 100 iterations. The function "f" is defined as "f = |13 * one(v) - 170|", where "one(v)" counts the number of ones in the 40-bit binary input variable "v".

In each iteration of the algorithm, a random 40-bit binary number is generated and used as the input for the function "f". The algorithm then calculates the value of "f" for this input and updates the current maximum value if the newly calculated value is greater. This process is repeated 100 times, and the maximum value of "f" found in each iteration is written to the "Output.txt" file. 

The output in the "Output.txt" file represents the result of the Hill-Climbing algorithm applied to the function "f". The maximum value of "f" found by the algorithm in each iteration can be used to analyze the performance of the algorithm and the quality of its solutions. The time and space complexities of the algorithm are also calculated and can be used to evaluate its efficiency and suitability for various use cases.


## Time and Space Complexities:
The time complexity of the hill-climbing algorithm is O(n), where n is the number of resets (in this case, n=100). The space complexity of the algorithm is O(1), as the algorithm only requires a constant amount of memory to store the best value and best v found during each reset.