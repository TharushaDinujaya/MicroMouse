#include <Arduino.h>
#include<maze.h>
#include<solver.h>

// create an array for represent the maze globally
struct Cell maze[MAZE_SIZE][MAZE_SIZE];
Orient orient = FORWARD;

// function definitions goes here
void initialize_maze();

void setup() {
  // board initializing routines goes here

  // array initialize routines goes here
  initialize_maze();
}

void loop() {
  // put your main code here, to run repeatedly:
}

void initialize_maze() {

  // fill the maze array with empty cells
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      maze[i][j].walls = WALL_NONE;
      maze[i][j].flood_id = UNKNOWN;
    }
  }

  // setup the boundary walls in the maze
  // bottom and top boundary walls
  for (int i = 0 ; i < MAZE_SIZE; i++) {
    maze[0][i].walls = maze[0][i].walls | WALL_BACK;
    maze[MAZE_SIZE - 1][i].walls = maze[MAZE_SIZE - 1][i].walls | WALL_FRONT;
  }

  // left and right boundary walls
  for (int i = 0; i < MAZE_SIZE; i++) {
    maze[i][0].walls = maze[i][0].walls | WALL_LEFT;
    maze[i][MAZE_SIZE - 1].walls = maze[i][MAZE_SIZE - 1].walls | WALL_RIGHT;
  }

}