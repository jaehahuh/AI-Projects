#include <iostream>
#include <fstream>
#include <ostream>
#include <string>

#define true 1
#define false 0
using namespace std;

// Function to find the first empty index found on the grid
int findBlank(int grid[6][6], int* row, int* col) {	
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			if (grid[i][j] == 0) {
				*row = i;
				*col = j;
				return true;
			}
		}
	}
	return false;
}

// Function to check the grid to see if the specified row already contains a number
int usedInRow(int grid[6][6], int row, int num) {	
	for (int col = 0; col < 6; col++) {
		if (grid[row][col] == num) return true;
	}
	return false;
}

// Function to check the grid to see if the specified column already contains a number
int usedInCol(int grid[6][6], int col, int num) {	
	for (int row = 0; row < 6; row++) {
		if (grid[row][col] == num) return true;
	}
	return false;
}


// inequality check function
int check(char h[6][5], char v[5][6], int grid[6][6]) {
	// Check for horizontal inequality.
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 5; j++) {
			// However, if at least one of the left and right spaces of the inequality sign is not padded, it is skipped without checking.
			if (grid[i][j] == 0 || grid[i][j + 1] == 0) continue;
			switch (h[i][j]) {
			case '>':
				if (grid[i][j] <= grid[i][j + 1]) return false;
				break;
			case '<':
				if (grid[i][j] >= grid[i][j + 1]) return false;
				break;
			default:
				break;
			}
		}
	}
	// Check for vertical inequality.
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 6; j++) {
			// However, if at least one of the up and down spaces of the inequality sign is not padded, it is skipped without checking.
			if (grid[i][j] == 0 || grid[i + 1][j] == 0) continue;
			switch (v[i][j]) {
			case '^':
				if (grid[i][j] >= grid[i + 1][j]) return false;
				break;
			case 'v':
				if (grid[i][j] <= grid[i + 1][j]) return false;
				break;
			default:
				break;
			}
		}
	}

	// Returns true if none of the above is detected.
	return true;
}

int isSafe(int grid[6][6], int row, int col, int num) {	
	// Check that the Futoshiki rule is satisfied when number is placed in the specified row and column.
	// The Futoshiki rule means that 1,2,3,4,5,6 should be used only once in horizontal and vertical lines.

	return !usedInRow(grid, row, num) && !usedInCol(grid, col, num);
}

//Problem solving function (recursive call)
int solve(char h[6][5], char v[5][6], int grid[6][6]) {	
	// Backtracking algorithm is used.
	// A method of proceeding by inserting the numbers 1-6 in order and checking whether the rules of the Futoshiki Puzzle are satisfied.
	
	int row, col;
	// Find the next blank.
	if (findBlank(grid, &row, &col) == 0) {
		return true;
	}

	for (int num = 1; num <= 6; num++) {
		// When number is in the corresponding cell, it checks whether 1,2,3,4,5,6 are duplicated in a row or column,
		if (isSafe(grid, row, col, num)) {
			// If not duplicated, add number.
			grid[row][col] = num;
			// Then check if the inequality sign is satisfied,
			if (!check(h, v, grid)) {
				// If not satisfied, it returns to zero.
				grid[row][col] = 0;
			} else {
				// If satisfied, fix it to number and recursively call solve for the next cell.
				if (solve(h, v, grid)) {
					return true;
				}
			}
			// If false is returned during recursion and has been backtracked to this side, it is returned to 0 and re-challenged to the next num.
			grid[row][col] = 0;
		}
	}

	// If you are here after all the loops are over, there is no correct answer.
	return false;
}

// Reads a character from the filestream that is not a blank or newline character.
char readChar(ifstream& file) {
	while (true) {
		char tmp;
		file.get(tmp);
		if (tmp != ' ' && tmp != '\n') return tmp;
	}
}
int main() {
	int grid[6][6];
	char h[6][5];
	char v[5][6];

	string filename;
	cout << "Input : ";
	cin >> filename;

	ifstream file;
	file.open(filename);

	// Read file
	if (file.is_open()) {
		// fill the grid first
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 6; j++) {
				file >> grid[i][j];
			}
		}

		// Fill in the horizontal inequality sign
		for (int i = 0; i < 6; i < i++) {
			for (int j = 0; j < 5; j++) {
				h[i][j] = readChar(file);
			}
		}

		//Fill in the vertical inequality sign
		for (int i = 0; i < 5; i < i++) {
			for (int j = 0; j < 6; j++) {
				v[i][j] = readChar(file);
			}
		}
	}
	else {
		cout << "File open failed." << endl;
	}

	file.close();

	// Get the name of the output file
	cout << "Output : ";
	cin >> filename;

	ofstream outFile;
	outFile.open(filename);

	if (solve(h, v, grid)) {	
		// When an answer is found, write the result to a file.
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 6; j++) {
				outFile << grid[i][j] << " ";
			}
			outFile << endl;
		}
	} else {
		// If you can't find an answer, write the sentence below to a file..
		outFile << "There is no solution!";
	}

	outFile.close();

	return 0;
}