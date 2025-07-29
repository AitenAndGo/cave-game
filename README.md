# Cave Game

A simple terminal-based game written in C++. You control a character using WASD keys in a cave-like environment. The goal is to provide a minimal but responsive game loop using only standard libraries.

## Features

- Responsive WASD input system
- 4-directional movement
- Terminal rendering using basic ASCII characters
- Lightweight and minimal dependencies

## Requirements

- C++ compiler (GCC or Clang)
- Linux/macOS terminal
- POSIX-compatible environment (for `termios` and `read`)

## Building

Clone the repository:

```bash
git clone https://github.com/AitenAndGo/cave-game.git
cd cave-game
```
```bash
g++ main.cpp -o cave-game
```
```bash
./cave-game
```

Use the following keys to move:

Key	Direction
W	Up
A	Left
S	Down
D	Right
Q	Quit game

License
This project is licensed under the MIT License.
