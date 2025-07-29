#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cfloat> 
#include <cmath>
#include <unistd.h>
#include <termios.h>
#include <chrono>
#include <thread>
#include <sys/ioctl.h>
#include <map>
#include <algorithm> // For std::swap and std::max/min
#include <list>      // Using list for efficient insertion/deletion of projectiles/enemies
#include <fcntl.h>

// ANSI color codes for better visuals
#define RESET   "\033[0m"
#define BLACK   "\033[30m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"
#define BRIGHT_BLACK   "\033[90m"
#define BRIGHT_RED     "\033[91m"
#define BRIGHT_GREEN   "\033[92m"
#define BRIGHT_YELLOW  "\033[93m"
#define BRIGHT_BLUE    "\033[94m"
#define BRIGHT_MAGENTA "\033[95m"
#define BRIGHT_CYAN    "\033[96m"
#define BRIGHT_WHITE   "\033[97m"

// Background colors
#define BG_BLACK   "\033[40m"
#define BG_RED     "\033[41m"
#define BG_GREEN   "\033[42m"
#define BG_YELLOW  "\033[43m"
#define BG_BLUE    "\033[44m"
#define BG_MAGENTA "\033[45m"
#define BG_CYAN    "\033[46m"
#define BG_WHITE   "\033[47m"

// Special characters for better visuals
#define WALL_CHAR "█"
#define PLAYER_CHAR "●"
#define GOAL_CHAR "★"
#define PATH_CHAR "·"
#define EMPTY_CHAR " "
#define ENEMY_CHAR "Ѫ"
#define PROJECTILE_CHAR "•"

// Helper Structures
class Vector2 {
public:
    int x;
    int y;
    Vector2(int _x = 0, int _y = 0) : x(_x), y(_y) {}
};

struct WindowSize {
    int width;
    int height;
};

struct Enemy {
    int x, y;
    enum State { IDLE, CHASING };
    State state = IDLE;
    int moveTimer = 0; // Timer to slow down enemy movement
};

struct Projectile {
    float x, y;
    float dx, dy; // Direction vector
};

// Function to get terminal window size
WindowSize getTerminalSize() {
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    return {w.ws_col, w.ws_row};
}

class InputHandler {
    private:
        std::vector<char> heldKeys;
        struct termios oldSettings, newSettings;
    
        bool isMovementKey(char ch) const {
            return ch == 'w' || ch == 'a' || ch == 's' || ch == 'd';
        }
    
        void addKey(char ch) {
            if (std::find(heldKeys.begin(), heldKeys.end(), ch) == heldKeys.end()) {
                heldKeys.push_back(ch);  // Add if not already in
            }
        }
    
        void removeKey(char ch) {
            heldKeys.erase(std::remove(heldKeys.begin(), heldKeys.end(), ch), heldKeys.end());
        }
    
    public:
        InputHandler() {
            tcgetattr(STDIN_FILENO, &oldSettings);
            newSettings = oldSettings;
            newSettings.c_lflag &= ~(ICANON | ECHO);
            newSettings.c_cc[VMIN] = 0;
            newSettings.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);
    
            fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
        }
    
        ~InputHandler() {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
        }
    
        void updateKeyStates() {
            heldKeys.clear();
            char ch;
            while (read(STDIN_FILENO, &ch, 1) > 0) {
                // Movement keys - toggle on
                if (isMovementKey(ch) || ch == ' ' || ch == 'q') {
                    addKey(ch);
                } else if (ch == '\033') {
                    // Handle escape or extended input if needed
                }
            }
    
            // Optional: remove keys if released — not possible with only `read` unless using OS-level input events.
            // This version assumes keys are held and manually released (e.g., time-based or by logic reset).
        }
    
        void simulateKeyRelease(char ch) {
            removeKey(ch);
        }
    
        Vector2 getMovement() const {
            float dx = 0, dy = 0;

            if (isHeld('w')) dy -= 1;
            if (isHeld('s')) dy += 1;
            if (isHeld('a')) dx -= 1;
            if (isHeld('d')) dx += 1;
        
            float length = std::sqrt(dx*dx + dy*dy);
            if (length == 0) return Vector2(0, 0);
            return Vector2(dx / length, dy / length);
        }
    
        bool isHeld(char key) const {
            return std::find(heldKeys.begin(), heldKeys.end(), key) != heldKeys.end();
        }
    
        bool shouldQuit() const { return isHeld('q'); }
        bool shootPressed() const { return isHeld(' '); }
    
        const std::vector<char>& getHeldKeys() const { return heldKeys; }
};

class cave{
public:
    std::vector<Vector2> Boundries;
};

// Maze Generation (unchanged from previous version)
class maze {
private:
    int size_x;
    int size_y;
    std::vector<cave> Caves;
    std::vector<std::vector<char>> map;
    std::vector<std::vector<char>> caveMap;
    Vector2 endPosition;

public:
    maze(int s = 10) : size_x(s), size_y(2*s), map(s, std::vector<char>(s*2, '#')) {}
    ~maze() {};

    void generateMaze(int noise = 50, int gen = 1){
        std::vector<std::vector<char>> bufferA = map;
        std::vector<std::vector<char>> bufferB = map;
        auto* current = &bufferA;
        auto* next = &bufferB;
        for (size_t i = 1; i < size_x-1; i++) {
            for (size_t j = 1; j < size_y-1; j++) {
                (*current)[i][j] = (rand() % 100 <= noise) ? '#' : ' ';
            }
        }
        static const int dx[8] = {-1,-1,-1,0,0,1,1,1};
        static const int dy[8] = {-1,0,1,-1,1,-1,0,1};
        for (int g = 0; g < gen; g++) {
            for (size_t i = 1; i < size_x-1; i++) {
                for (size_t j = 1; j < size_y-1; j++) {
                    int wallCount = 0;
                    for (int d = 0; d < 8; ++d) {
                        if ((*current)[i+dx[d]][j+dy[d]] == '#') wallCount++;
                    }
                    (*next)[i][j] = (wallCount >= 4) ? '#' : ' ';
                }
            }
            std::swap(current, next);
        }
        map = *current;
        map[size_x / 2][size_y / 2] = ' ';
        int margin = 5;
        int maxDistanceFromEdge = 20;
        int dir_x = rand() % 2;
        int dir_y = rand() % 2;
        if (dir_x == 0){ dir_x = -1; } else{ dir_x = 1; }
        if (dir_y == 0){ dir_y = -1; } else{ dir_y = 1; }
        endPosition = Vector2(
            size_x / 2 + (dir_x * (size_x / 2 - margin - rand() % maxDistanceFromEdge)),
            size_y / 2 + (dir_y * (size_y / 2 - margin - rand() % maxDistanceFromEdge))
        );
        map[endPosition.x][endPosition.y] = ' ';
    }

    void drawLineBresenham(Vector2 start, Vector2 end) {
        int x1 = start.x, y1 = start.y;
        int x2 = end.x, y2 = end.y;
        if (x1 == 0 || y1 == 0) return;
        int dx = std::abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
        int dy = -std::abs(y2 - y1), sy = y1 < y2 ? 1 : -1; 
        int err = dx + dy, e2; 
        while (true) {
            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    int nx = x1 + i, ny = y1 + j;
                    if (nx > 0 && nx < size_x - 1 && ny > 0 && ny < size_y - 1) {
                        map[nx][ny] = ' ';
                    }
                }
            }
            if (x1 == x2 && y1 == y2) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x1 += sx; } 
            if (e2 <= dx) { err += dx; y1 += sy; }
        }
    }
    
    void connectCaves(){
        findCaves();
        if (Caves.size() < 2) {
            if (endPosition.x != 0 && endPosition.y != 0)
                map[endPosition.x][endPosition.y] = '$';
            return;
        }
        int largestCaveIndex = 0;
        size_t maxCaveSize = 0;
        for (size_t i = 0; i < Caves.size(); ++i) {
            if (Caves[i].Boundries.size() > maxCaveSize) {
                maxCaveSize = Caves[i].Boundries.size();
                largestCaveIndex = i;
            }
        }
        std::swap(Caves[0], Caves[largestCaveIndex]);
        for (size_t i = 1; i < Caves.size(); ++i) {
            float minDistance = FLT_MAX;
            Vector2 startPoint, endPoint;
            for (const auto& p1 : Caves[0].Boundries) {
                for (const auto& p2 : Caves[i].Boundries) {
                    float dist = powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2);
                    if (dist < minDistance) {
                        minDistance = dist;
                        startPoint = p1;
                        endPoint = p2;
                    }
                }
            }
            drawLineBresenham(startPoint, endPoint);
            Caves[0].Boundries.insert(Caves[0].Boundries.end(), Caves[i].Boundries.begin(), Caves[i].Boundries.end());
        }
        map[endPosition.x][endPosition.y] = '$';
    }

    void findCaves(){
        Caves.clear();
        caveMap = map;
        char index = '0';
        for (int i = 1; i < size_x - 1; i++){
            for (int j = 1; j < size_y - 1; j++){
                if (caveMap[i][j] == ' '){
                    cave newCave;
                    fillCave(i, j, index, newCave);
                    if (!newCave.Boundries.empty()) {
                        Caves.push_back(newCave);
                    }
                    index++;
                }
            }
        }
    }

    void fillCave(int x, int y, int index, cave& NewCave){
        if (x <= 0 || x >= size_x - 1 || y <= 0 || y >= size_y - 1 || caveMap[x][y] != ' ') {
            return;
        }
        caveMap[x][y] = index;
        if (map[x + 1][y] == '#' || map[x - 1][y] == '#' || map[x][y + 1] == '#' || map[x][y - 1] == '#'){
            NewCave.Boundries.push_back(Vector2(x, y));
        }
        fillCave(x + 1, y, index, NewCave);
        fillCave(x - 1, y, index, NewCave);
        fillCave(x, y + 1, index, NewCave);
        fillCave(x, y - 1, index, NewCave);
    }

    std::vector<std::vector<char>> getMap() { return map; }
};

// Renderer
class ARES {
public:
    void draw(const std::vector<std::vector<char>>& map, 
              const std::vector<std::vector<bool>>& visibleMap,
              const std::list<Enemy>& enemies,
              const std::list<Projectile>& projectiles,
              int playerX, int playerY, int moves, int enemyCount) {
        WindowSize window = getTerminalSize();
        int mapHeight = map.size();
        int mapWidth = map[0].size();
        
        int viewSizeX = std::min((int)map[0].size(), window.width - 4);
        int viewSizeY = std::min((int)map.size(), window.height - 10);

        int viewX = std::max(0, std::min(playerX - viewSizeX / 2, mapWidth - viewSizeX));
        int viewY = std::max(0, std::min(playerY - viewSizeY / 2, mapHeight - viewSizeY));

        std::cout << "\033[H";

        // Top border
        std::cout << BRIGHT_CYAN << "╔";
        for (int i = 0; i < viewSizeX; i++) std::cout << "═";
        std::cout << "╗" << RESET << "\n";
        
        for (int y = 0; y < viewSizeY; ++y) {
            std::cout << BRIGHT_CYAN << "║" << RESET;
            for (int x = 0; x < viewSizeX; ++x) {
                int map_x = viewX + x;
                int map_y = viewY + y;

                if (visibleMap[map_y][map_x]) {
                    bool isDrawn = false;
                    // Draw player
                    if (map_x == playerX && map_y == playerY) {
                        std::cout << BRIGHT_YELLOW << PLAYER_CHAR << RESET;
                        isDrawn = true;
                    }
                    // Draw enemies
                    if (!isDrawn) {
                        for (const auto& enemy : enemies) {
                            if (enemy.x == map_x && enemy.y == map_y) {
                                std::cout << BRIGHT_RED << ENEMY_CHAR << RESET;
                                isDrawn = true;
                                break;
                            }
                        }
                    }
                    // Draw projectiles
                    if (!isDrawn) {
                        for (const auto& p : projectiles) {
                            if (static_cast<int>(p.x) == map_x && static_cast<int>(p.y) == map_y) {
                                std::cout << BRIGHT_YELLOW << PROJECTILE_CHAR << RESET;
                                isDrawn = true;
                                break;
                            }
                        }
                    }
                    // Draw map elements
                    if (!isDrawn) {
                        char cell = map[map_y][map_x];
                        if (cell == '#') std::cout << BG_BLUE << BRIGHT_WHITE << WALL_CHAR << RESET;
                        else if (cell == '$') std::cout << BRIGHT_RED << GOAL_CHAR << RESET;
                        else if (cell == ' ') std::cout << BG_BLACK << BRIGHT_BLACK << PATH_CHAR << RESET;
                        else std::cout << cell;
                    }
                } else {
                    std::cout << EMPTY_CHAR; // Fog of war
                }
            }
            std::cout << BRIGHT_CYAN << "║" << RESET << "\n";
        }
        
        // Bottom border
        std::cout << BRIGHT_CYAN << "╚";
        for (int i = 0; i < viewSizeX; i++) std::cout << "═";
        std::cout << "╝" << RESET << "\n";
        
        // UI Info
        std::cout << "Controls: " << RESET << "WASD (move), SPACE (shoot), Q (quit)\n";
        std::cout << "Goal: " << RESET << "Find the " << BRIGHT_RED << GOAL_CHAR << RESET << " or eliminate all enemies!\n";
        std::cout << BRIGHT_CYAN << "Moves: " << RESET << moves << " | " << BRIGHT_RED << "Enemies Left: " << RESET << enemyCount << "\n";
        std::cout.flush();
    }
};

// Main Game Logic
class Game {
    std::vector<std::vector<char>> map;
    std::vector<std::vector<char>> originalMap;
    std::vector<std::vector<bool>> visibleMap;
    int playerX, playerY;
    int moves;
    bool gameWon, gameOver;
    InputHandler inputHandler;
    ARES renderer;
    Vector2 lastMoveDir = {1, 0};
    
    std::list<Enemy> enemies;
    std::list<Projectile> projectiles;

    const int sightRadius = 15;
    const int ENEMY_MOVE_DELAY = 6;

public:
    Game(int mapSize, std::vector<std::vector<char>> _map, int enemyCount) : 
        map(_map), originalMap(_map), moves(0), gameWon(false), gameOver(false),
        visibleMap(_map.size(), std::vector<bool>(_map[0].size(), false)) {
        
        // Place player
        do {
            playerX = map[0].size() / 2 + (rand() % 20 - 10);
            playerY = map.size() / 2 + (rand() % 10 - 5);
        } while(map[playerY][playerX] == '#');
        
        // Place enemies
        for (int i = 0; i < enemyCount; ++i) {
            Enemy enemy;
            do {
                enemy.x = rand() % map[0].size();
                enemy.y = rand() % map.size();
            } while (map[enemy.y][enemy.x] == '#' || (abs(enemy.x - playerX) < 20 && abs(enemy.y - playerY) < 20));
            enemies.push_back(enemy);
        }
    }

    // --- Core Game Loop Functions ---
    int run() {
        inputHandler.updateKeyStates();
        if (inputHandler.shouldQuit() || gameOver) return 0;
        if (gameWon) return 1;

        updatePlayer();
        updateProjectiles();
        updateEnemies();
        calculateFOV();
        
        checkWinConditions();
        std::cout.flush();
        // inputHandler.resetKeyStates();
        return 1;
    }

    void draw() {
        renderer.draw(map, visibleMap, enemies, projectiles, playerX, playerY, moves, enemies.size());
    }

private:
    // --- Update Logic ---
    void updatePlayer() {
        Vector2 movement = inputHandler.getMovement();
        if (movement.x != 0 || movement.y != 0) {
            lastMoveDir = movement;
            int newX = playerX + movement.x;
            int newY = playerY + movement.y;
            if (newX >= 0 && newX < map[0].size() && newY >= 0 && newY < map.size() && map[newY][newX] != '#') {
                playerX = newX;
                playerY = newY;
                moves++;
            }
        }

        if (inputHandler.shootPressed()) {
            Projectile p;
            p.x = playerX;
            p.y = playerY;
            float len = std::sqrt(lastMoveDir.x * lastMoveDir.x + lastMoveDir.y * lastMoveDir.y);
            if (len > 0) {
                p.dx = lastMoveDir.x / len * 0.8f; // Speed
                p.dy = lastMoveDir.y / len * 0.8f;
                projectiles.push_back(p);
            }
        }
    }

    void updateProjectiles() {
        for (auto it = projectiles.begin(); it != projectiles.end(); ) {
            it->x += it->dx;
            it->y += it->dy;
            int ix = static_cast<int>(it->x);
            int iy = static_cast<int>(it->y);

            bool removed = false;
            if (ix <= 0 || ix >= map[0].size() -1 || iy <= 0 || iy >= map.size() -1 || map[iy][ix] == '#') {
                it = projectiles.erase(it);
                removed = true;
            }

            if (!removed) {
                for (auto enemy_it = enemies.begin(); enemy_it != enemies.end(); ) {
                    if (enemy_it->x == ix && enemy_it->y == iy) {
                        enemy_it = enemies.erase(enemy_it);
                        it = projectiles.erase(it);
                        removed = true;
                        break; 
                    } else {
                        ++enemy_it;
                    }
                }
            }
            if (!removed) ++it;
        }
    }

    void updateEnemies() {
        for (auto& enemy : enemies) {
            if (enemy.x == playerX && enemy.y == playerY) {
                gameOver = true;
                loseScreen();
                return;
            }

            float distanceToPlayer = std::sqrt(pow(enemy.x - playerX, 2) + pow(enemy.y - playerY, 2));
            
            if (distanceToPlayer < 15 && hasLineOfSight(enemy.x, enemy.y, playerX, playerY)) {
                enemy.state = Enemy::CHASING;
            } else {
                enemy.state = Enemy::IDLE;
            }

            if (enemy.state == Enemy::CHASING) {
                enemy.moveTimer++;
                if (enemy.moveTimer >= ENEMY_MOVE_DELAY) {
                    enemy.moveTimer = 0; // Reset timer

                    int moveX = 0, moveY = 0;
                    if (playerX < enemy.x) moveX = -1; else if (playerX > enemy.x) moveX = 1;
                    if (playerY < enemy.y) moveY = -1; else if (playerY > enemy.y) moveY = 1;
                    
                    if (moveX != 0 && moveY != 0 && map[enemy.y + moveY][enemy.x] == '#' && map[enemy.y][enemy.x + moveX] == '#') {
                        // Stuck in a corner, try moving only one way
                        if (rand() % 2 == 0) moveY = 0; else moveX = 0;
                    }

                    if (map[enemy.y + moveY][enemy.x + moveX] != '#') { enemy.x += moveX; enemy.y += moveY; }
                    else if (map[enemy.y][enemy.x + moveX] != '#') { enemy.x += moveX; }
                    else if (map[enemy.y + moveY][enemy.x] != '#') { enemy.y += moveY; }
                }
            }
        }
    }

    // --- Visibility and Line of Sight ---
    void castRay(int x0, int y0, int x1, int y1) {
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        while (true) {
            if (x0 < 0 || x0 >= map[0].size() || y0 < 0 || y0 >= map.size()) break;
            visibleMap[y0][x0] = true;
            if (map[y0][x0] == '#') break;
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;

            // Diagonal wall check
            if (e2 >= dy && e2 <= dx) {
                if (map[y0][x0 + sx] == '#' && map[y0 + sy][x0] == '#') {
                    break; // Block sight at diagonal corners
                }
            }

            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    void calculateFOV() {
        for(auto& row : visibleMap) std::fill(row.begin(), row.end(), false);
        for (int angle = 0; angle < 360; angle += 1) {
            int x1 = playerX + static_cast<int>(sightRadius * cos(angle * M_PI / 180.0));
            int y1 = playerY + static_cast<int>(sightRadius * sin(angle * M_PI / 180.0));
            castRay(playerX, playerY, x1, y1);
        }
    }

    bool hasLineOfSight(int x0, int y0, int x1, int y1) {
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        while (true) {
            if (map[y0][x0] == '#') return false; // Blocked by current cell
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            
            // Diagonal wall check
            if (e2 >= dy && e2 <= dx) {
                if (map[y0][x0 + sx] == '#' && map[y0 + sy][x0] == '#') {
                    return false; // Blocked by diagonal corner
                }
            }

            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
        return true; // Not blocked
    }

    // --- Win/Loss Conditions ---
    void checkWinConditions() {
        if (map[playerY][playerX] == '$' || enemies.empty()) {
            gameWon = true;
            winScreen();
        }
    }

    void winScreen() {
        std::cout << "\033[2J\033[H";
        std::cout << BRIGHT_GREEN << R"(
 __     __          __          ___       _ 
 \ \   / /          \ \        / (_)     | |
  \ \_/ /__  _   _   \ \  /\  / / _ _ __ | |
   \   / _ \| | | |   \ \/  \/ / | | '_ \| |
    | | (_) | |_| |    \  /\  /  | | | | |_|
    |_|\___/ \__,_|     \/  \/   |_|_| |_(_)
)" << RESET << std::endl;
        std::cout << BRIGHT_YELLOW << "\n      You found the exit and escaped the maze!" << RESET << std::endl;
        std::cout << BRIGHT_CYAN << "\n         🏆 MAZE COMPLETED! 🏆" << RESET << std::endl;
        std::cout << BRIGHT_WHITE << "\n\n     Press Q or Enter to exit..." << RESET << std::endl;
        // Wait for Q or Enter
        char c = 0;
        struct termios oldSettings, newSettings;
        tcgetattr(STDIN_FILENO, &oldSettings);
        newSettings = oldSettings;
        newSettings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);
        while (c != 'q' && c != 'Q' && c != '\n' && c != '\r') {
            read(STDIN_FILENO, &c, 1);
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    }

    void loseScreen() {
        std::cout << "\033[2J\033[H";
        std::cout << BRIGHT_RED << R"(
   _____                         ____                 
  / ____|                       / __ \                
 | |  __  __ _ _ __ ___   ___  | |  | |_   _____ _ __ 
 | | |_ |/ _` | '_ ` _ \ / _ \ | |  | \ \ / / _ \ '__|
 | |__| | (_| | | | | | |  __/ | |__| |\ V /  __/ |   
  \_____|\__,_|_| |_| |_|\___|  \____/  \_/ \___|_|   
)" << RESET << std::endl;
        std::cout << BRIGHT_YELLOW << "\n      An enemy caught you before you could escape!" << RESET << std::endl;
        std::cout << BRIGHT_CYAN << "\n         💀 GAME OVER 💀" << RESET << std::endl;
        std::cout << BRIGHT_WHITE << "\n\n     Press Q or Enter to exit..." << RESET << std::endl;
        // Wait for Q or Enter
        char c = 0;
        struct termios oldSettings, newSettings;
        tcgetattr(STDIN_FILENO, &oldSettings);
        newSettings = oldSettings;
        newSettings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);
        while (c != 'q' && c != 'Q' && c != '\n' && c != '\r') {
            read(STDIN_FILENO, &c, 1);
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    }
};

void showLoadingAnimation() {
    std::cout << BRIGHT_CYAN << "\nGenerating your maze";
    for (int i = 0; i < 3; i++) {
        std::cout << ".";
        std::cout.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    std::cout << RESET << std::endl;
}

int startScreen() {
    std::cout << "\033[2J\033[H";
    std::cout << BRIGHT_CYAN << "╔══════════════════════════════════════════════════════════╗" << RESET << "\n";
    std::cout << BRIGHT_CYAN << "║" << RESET << BRIGHT_GREEN << "               WELCOME TO MAZE ADVENTURE                " << RESET << BRIGHT_CYAN << "  ║" << RESET << "\n";
    std::cout << BRIGHT_CYAN << "╚══════════════════════════════════════════════════════════╝" << RESET << "\n\n";
    std::cout << BRIGHT_YELLOW << "🎮 Controls:" << RESET << "\n";
    std::cout << "   " << BRIGHT_WHITE << "W, A, S, D" << RESET << " - Move (hold for continuous movement)\n";
    std::cout << "   " << BRIGHT_WHITE << "SPACE" << RESET << "      - Shoot\n";
    std::cout << "   " << BRIGHT_RED << "Q" << RESET << "          - Quit Game\n\n";
    std::cout << BRIGHT_MAGENTA << "🎯 Objective:" << RESET << " Find the " << BRIGHT_RED << GOAL_CHAR << RESET << " or defeat all enemies!\n\n";
    std::cout << BRIGHT_CYAN << "Select difficulty level:" << RESET << "\n";
    std::cout << BRIGHT_GREEN << "1. " << RESET << "Easy   " << BRIGHT_BLACK << "(Small map, 5 enemies)" << RESET << "\n";
    std::cout << BRIGHT_YELLOW << "2. " << RESET << "Medium " << BRIGHT_BLACK << "(Medium map, 10 enemies)" << RESET << "\n";
    std::cout << BRIGHT_RED << "3. " << RESET << "Hard   " << BRIGHT_BLACK << "(Large map, 20 enemies)" << RESET << "\n";
    std::cout << BRIGHT_CYAN << "\nEnter your choice (1/2/3): " << RESET;
    
    char choice;
    struct termios oldSettings, newSettings;
    tcgetattr(STDIN_FILENO, &oldSettings);
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);
    while (true) {
        read(STDIN_FILENO, &choice, 1);
        if (choice >= '1' && choice <= '3') break;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    system("clear");
    return choice - '0';
}

int main() {
    srand(static_cast<unsigned int>(time(0)));

    int difficulty = startScreen();
    int mapSize, enemyCount;
    std::string difficultyName;

    if (difficulty == 1) { mapSize = 60; enemyCount = 8; difficultyName = "Easy"; const int ENEMY_MOVE_DELAY = 6;} 
    else if (difficulty == 2) { mapSize = 120; enemyCount = 16; difficultyName = "Medium"; const int ENEMY_MOVE_DELAY = 5;} 
    else { mapSize = 200; enemyCount = 30; difficultyName = "Hard"; const int ENEMY_MOVE_DELAY = 4;}
    
    system("clear");
    std::cout << BRIGHT_CYAN << "🎲 Difficulty: " << RESET << difficultyName << std::endl;
    showLoadingAnimation();

    maze Cave(mapSize);
    Cave.generateMaze(42, 5); // Slightly more open caves
    Cave.connectCaves();
    system("clear");

    Game game(mapSize, Cave.getMap(), enemyCount);

    while (true) {
        game.draw();
        if (game.run() == 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
