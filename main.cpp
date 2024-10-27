#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cfloat> 
#include <cmath>
#include <unistd.h>
#include <termios.h>

class Vector2 {
public:
    int x;
    int y;

    Vector2(int _x = 0, int _y = 0) {
        x = _x;
        y = _y;
    }
};

class cave{
public:
    std::vector<Vector2> Boundries;
};

class maze {
private:
    int size_x;
    int size_y;
    Vector2 start;
    Vector2 end;

    std::vector<cave> Caves;

    std::vector<std::vector<char>> map;
    std::vector<std::vector<char>> caveMap;

    Vector2 endPosition;

public:
    maze(int s = 10) : size_x(s), size_y(2*s), map(s, std::vector<char>(s*2, '#')) {
        start = Vector2(0, 0);
        end = Vector2(size_x - 1, size_y - 1);
        map[start.x][start.y] = 'S';
        map[end.x][end.y] = 'E';
    }

    ~maze() {};

    void generateMaze(int noise = 50, int gen = 1){

        for (size_t i = 1; i < size_x-1; i++)
        {
            for (size_t j = 1; j < size_y-1; j++)
            {
                if (rand() % 100 <= noise){
                    map[i][j] = '#';
                } else{
                    map[i][j] = ' ';
                }
            }
        }

        for (int g = 0; g < gen; g++){

            std::vector<std::vector<char>> Newmap = map;

            for (size_t i = 1; i < size_x-1; i++)
            {
                for (size_t j = 1; j < size_y-1; j++)
                {
                    int wallCount = 0;
                    for (int x = i - 1; x <= i + 1; x++){
                        for (int y = j - 1; y <= j + 1; y++){
                            if (x != i || y != j){
                                if (Newmap[x][y] == '#'){
                                    wallCount++;
                                }
                            }
                        }
                    }
                    if (wallCount >= 4){
                        map[i][j] = '#';
                    } else{
                        map[i][j] = ' ';
                    }
                }
            }
        }
        map[size_x / 2][size_y / 2] = ' ';
        
        int margin = 5;
        int maxDistanceFromEdge = 20;

        int dir_x = rand() % 2;
        int dir_y = rand() % 2;
        if (dir_x == 0){
            dir_x = -1;
        } else{
            dir_x = 1;
        }

        if (dir_y == 0){
            dir_y = -1;
        } else{
            dir_y = 1;
        }

        endPosition = Vector2(
            size_x / 2 + (dir_x * (size_x / 2 - margin - rand() % maxDistanceFromEdge)),
            size_y / 2 + (dir_y * (size_y / 2 - margin - rand() % maxDistanceFromEdge))
        );

        map[endPosition.x][endPosition.y] = ' ';
    }

    void drawLineBresenham(Vector2 start, Vector2 end) {
        int x1 = start.x;
        int y1 = start.y;
        int x2 = end.x;
        int y2 = end.y;

        if (x1 == 0 || y1 == 0){
            return;
        }

        int dx = std::abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
        int dy = -std::abs(y2 - y1), sy = y1 < y2 ? 1 : -1; 
        int err = dx + dy, e2; // błąd dla algorytmu Bresenhama

        while (true) {
            // Rysowanie przejścia (znak '#') w caveMap
            map[x1][y1] = ' ';
            map[x1 + 1][y1] = ' ';
            map[x1 - 1][y1] = ' ';
            map[x1][y1 + 1] = ' ';
            map[x1][y1 - 1] = ' ';
            map[x1 + 1][y1 + 1] = ' ';
            map[x1 + 1][y1 - 1] = ' ';
            map[x1 - 1][y1 + 1] = ' ';
            map[x1 - 1][y1 - 1] = ' ';

            // Sprawdzenie czy osiągnęliśmy punkt końcowy
            if (x1 == x2 && y1 == y2) break;

            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x1 += sx; } // poruszanie się w kierunku x
            if (e2 <= dx) { err += dx; y1 += sy; } // poruszanie się w kierunku y
        }
    }

    void connectCaves(){
        findCaves();
        while (Caves.size() >= 1)
        {
            float distance = FLT_MAX;
            int combineCaveIndex = 0;
            Vector2 start;
            Vector2 end;
            for (int i = 0; i < Caves[0].Boundries.size(); i++){
                for(int c = 1; c < Caves.size(); c++){
                    for (int j = 0; j < Caves[c].Boundries.size(); j++){
                        float tempDist = std::sqrt(powf((Caves[0].Boundries[i].x - Caves[c].Boundries[j].x), 2) + powf((Caves[0].Boundries[i].y - Caves[c].Boundries[j].y), 2));
                        if (tempDist <= distance){
                            distance = tempDist;
                            combineCaveIndex = c;
                            start = Vector2(Caves[0].Boundries[i].x, Caves[0].Boundries[i].y);
                            end = Vector2(Caves[c].Boundries[j].x, Caves[c].Boundries[j].y);
                        }
                    }
                }
            }
            Caves[0].Boundries.insert(Caves[0].Boundries.end(), Caves[combineCaveIndex].Boundries.begin(), Caves[combineCaveIndex].Boundries.end());
            Caves.erase(Caves.begin() + combineCaveIndex);

            // make connections
            drawLineBresenham(start, end);
        }

        // for (int i = 0; i < Caves[0].Boundries.size(); i++){
        //     caveMap[Caves[0].Boundries[i].x][Caves[0].Boundries[i].y] = ' ';
        //     // map = caveMap;
        // }

        map[endPosition.x][endPosition.y] = '$';
        
    }

    void findCaves(){
        caveMap = map;
        char index = '0';

        for (int i = 1; i < size_x - 1; i++){
            for (int j = 1; j < size_y - 1; j++){
                if (caveMap[i][j] == ' '){
                    cave newCave;
                    fillCave(i, j, index, newCave);
                    Caves.push_back(newCave);
                    index++;
                }
            }
        }
    }

    void fillCave(int x, int y, int index, cave& NewCave){
        caveMap[x][y] = index;

        if (caveMap[x + 1][y] == '#' ||
            caveMap[x - 1][y] == '#' ||
            caveMap[x][y + 1] == '#' ||
            caveMap[x][y - 1] == '#'){
            NewCave.Boundries.push_back(Vector2(x, y));
        }

        if (caveMap[x + 1][y] == ' '){
            fillCave(x + 1, y, index, NewCave);
        } 
        if (caveMap[x - 1][y] == ' '){
            fillCave(x-1, y, index, NewCave);
        }
        if (caveMap[x][y + 1] == ' '){
            fillCave(x, y + 1, index, NewCave);
        }
        if (caveMap[x][y - 1] == ' '){
            fillCave(x, y - 1, index, NewCave);
        }
    }

    std::vector<std::vector<char>> getMap() {
        return map;
    }
};


// Funkcja do wczytywania pojedynczego znaku bez potrzeby naciskania Enter
char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
    old.c_lflag &= ~ICANON; // Wyłącza kanoniczny tryb (bez Enter)
    old.c_lflag &= ~ECHO;   // Wyłącza wyświetlanie znaku
    if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0) perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) perror("tcsetattr ~ICANON");
    return buf;
}

class ARES {
public:
    void draw(const std::vector<std::vector<char>>& map, int viewX, int viewY) {
        int viewSize = 30; // Rozmiar widoku 30x30
        std::cout << "\033[H"; // Przesunięcie kursora na górę terminala
        for (int y = 0; y < viewSize; ++y) {
            for (int x = 0; x < viewSize * 2; ++x) {
                if (map[viewY + y][viewX + x] == '#') {
                // Ustawienie białego tła i pełnego bloku jako ściany
                std::cout << "\033[47m█\033[0m";
                } else {
                    // Zwykły znak dla innych elementów mapy
                    std::cout << map[viewY + y][viewX + x];
                }
            }
            std::cout << "\n";
        }
        std::cout.flush();
    }
};

class Game {
    std::vector<std::vector<char>> map;
    std::vector<std::vector<char>> originalMap;
    int playerX, playerY;
    int viewX, viewY;
    int viewSizeX;
    int viewSizeY;

public:
    Game(int mapSize, int viewSize, std::vector<std::vector<char>> _map) : viewSizeX(2*viewSize), viewSizeY(viewSize), map(_map), originalMap(_map) {
        playerX = mapSize;
        playerY = mapSize / 2;
        map[playerY][playerX] = 'O';

        viewX = std::max(0, playerX - viewSize);
        viewY = std::max(0, playerY - viewSize / 2);
    }

    int update(char input) {

        if (input == 'q'){
            return 0;
        }

        map[playerY][playerX] = originalMap[playerY][playerX]; // Zapis poprawny z `playerY`

        // Aktualizacja pozycji gracza
        if (input == 'w' && playerY > 0 && map[playerY - 1][playerX] != '#') playerY--;
        else if (input == 's' && playerY < map.size() - 1 && map[playerY + 1][playerX] != '#') playerY++;
        else if (input == 'a' && playerX > 0 && map[playerY][playerX - 1] != '#') playerX--;
        else if (input == 'd' && playerX < map[0].size() - 1 && map[playerY][playerX + 1] != '#') playerX++;

        if (map[playerY][playerX] == '$'){
            winScreen();
        }

        map[playerY][playerX] = 'O';

        // Przesunięcie widoku
        viewX = std::max(0, std::min(playerX - viewSizeX / 2, (int)map.size() * 2 - viewSizeX));
        viewY = std::max(0, std::min(playerY - viewSizeY / 2, (int)map.size() - viewSizeY));
        return 1;
    }

   void winScreen() {
    std::cout << "\033[2J\033[H"; // Wyczyszczenie ekranu i ustawienie kursora na górze
    std::cout << R"(
 __     ______  _    _  __          _______ _   _ 
 \ \   / / __ \| |  | | \ \        / /_   _| \ | |
  \ \_/ / |  | | |  | |  \ \  /\  / /  | | |  \| |
   \   /| |  | | |  | |   \ \/  \/ /   | | | . ` |
    | | | |__| | |__| |    \  /\  /   _| |_| |\  |
    |_|  \____/ \____/      \/  \/   |_____|_| \_|







                                               
    )" << std::endl;

    // Krótkie opóźnienie, aby użytkownik zobaczył ekran wygranej
    sleep(3); // Czas w sekundach
}

    std::vector<std::vector<char>> getView() const {
        std::vector<std::vector<char>> view(viewSizeY, std::vector<char>(viewSizeX, '.'));
        for (int y = 0; y < viewSizeY; ++y) {
            for (int x = 0; x < viewSizeX; ++x) {
                view[y][x] = map[viewY + y][viewX + x];
            }
        }
        return view;
    }
};

int startScreen() {
    std::cout << "\033[2J\033[H"; // Czyszczenie ekranu i ustawienie kursora na górze
    std::cout << "====================\n";
    std::cout << "      WELCOME       \n";
    std::cout << "====================\n\n";
    std::cout << "Controls:\n";
    std::cout << "Move with W, A, S, D\n";
    std::cout << "Find the $ symbol to win!\n";
    std::cout << "Press Q to quit the game\n\n";
    
    std::cout << "Select difficulty level:\n";
    std::cout << "1. Easy\n";
    std::cout << "2. Medium\n";
    std::cout << "3. Hard\n";
    std::cout << "Enter your choice (1/2/3): ";
    
    char choice;
    while (true) {
        choice = getch();
        if (choice == '1' || choice == '2' || choice == '3') {
            system("clear");
            break;
        }
    }

    return choice - '0';
}


int main() {
    srand(static_cast<unsigned int>(time(0)));

    ARES window;
    int difficulty = startScreen();

    int mapSize;
    if (difficulty == 1) {
        mapSize = 45; // Łatwy poziom
    } else if (difficulty == 2) {
        mapSize = 120; // Średni poziom
    } else {
        mapSize = 200; // Trudny poziom
    }
    system("clear");
    std::cout << "\nWait, map is being generated, it could take a minute! :D" << std::endl;
    sleep(3);

    maze Cave(mapSize);
    Cave.generateMaze(39, 7);
    Cave.connectCaves();
    system("clear");

    Game game(mapSize, 30, Cave.getMap());

    while (true) {
        window.draw(game.getView(), 0, 0);
        char input = getch();
        if (game.update(input) == 0) {
            break;
        }
    }

    return 0;
}
