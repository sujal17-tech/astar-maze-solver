# Maze Solver using A* Search

## Overview
This project implements the A* search algorithm to solve mazes represented as a grid of walkable cells and walls. It models each cell as a node with cost values (g, h, f) and supports both Manhattan and Euclidean heuristics to guide the search efficiently toward the goal. The algorithm returns the shortest path when one exists, and correctly detects and reports when the goal is unreachable rather than failing silently. Both a console-based text view and a matplotlib visualization are included to display explored cells and the final path. Built as Task 1 for my AI/ML internship to apply core pathfinding concepts in a hands-on, visual way.

## Features
- A* pathfinding
- Manhattan & Euclidean heuristics
- Console visualization
- Matplotlib visualization
- Detects unreachable goals

## Project Structure
maze_astar/
├── maze_solver.py
└── README.md

## Requirements
- Python 3.9+
- matplotlib

## Installation
pip install matplotlib

## Run
python maze_solver.py

## How it Works
Short explanation of:
- Node
- Maze
- A* algorithm
- Heuristics

## Example Output

/usr/bin/python3 "/Users/sujalbhedurkar/Desktop/CODING/Python/Maze Solver using A*.py"
sujalbhedurkar@Sujals-MacBook-Air CODING % /usr/bin/python3 
"/Users/sujalbhedurk
ar/Desktop/CODING/Py
thon/Maze Solver usi
ng A*.py"
============================================================
A* MAZE SOLVER DEMO
============================================================

[Case 1] Solvable maze
Start: (0, 0)  Goal: (9, 9)
Path found! Length = 19 steps, cost = 18, time = 0.0001s
Nodes explored: 42
Path: [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (0, 7), (0, 8), (0, 9), (1, 9), (2, 9), (3, 9), (4, 9), (5, 9), (6, 9), (7, 9), (8, 9), (9, 9)]

S * * * * * * * * * 
o # # # # # # # # * 
o # . . . . . . # * 
o # . # # # # . # * 
o o o # . . # . # * 
o # # # . . # . # * 
o # . . . # # . # * 
o # . # # # . . # * 
o o o o o o o # . * 
o o o o o o o # . G 

[!] matplotlib not installed -- falling back to console view.

S * * * * * * * * * 
o # # # # # # # # * 
o # . . . . . . # * 
o # . # # # # . # * 
o o o # . . # . # * 
o # # # . . # . # * 
o # . . . # # . # * 
o # . # # # . . # * 
o o o o o o o # . * 
o o o o o o o # . G 


[Case 2] Unreachable goal
Start: (0, 0)  Goal: (0, 4)
Correctly detected: NO PATH EXISTS (goal unreachable).

S o o # G 
o # o # . 
o # o # . 
o # o # . 
o o o # . 

[!] matplotlib not installed -- falling back to console view.

S o o # G 
o # o # . 
o # o # . 
o # o # . 
o o o # . 

Done. Check 'solvable_maze_result.png' and 'unreachable_maze_result.png' for saved plots.
sujalbhedurkar@Sujals-MacBook-Air CODING % 

## Technologies Used
- Python
- heapq
- matplotlib
- math

## Future Improvements
- Random maze generation
- GUI
- Animation
- File input

## Author
Sujal Bhedurkar
