#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <fstream>
#include <sstream>
#include <iostream>
#include <functional>
#include <memory>
#include <stack>
#include <chrono>

// структура для представления узла графа
struct Node
{
    double lon, lat;                              // долгота и широта узла
    std::vector<std::pair<Node *, double>> edges; // список рёбер, исходящих из узла
};

// хэш-функция для пар (долгота, широта) для использования в unordered_map
struct pair_hash
{
    // шаблонный оператор для вычисления хэша пары значений
    template <class T1, class T2>
    size_t operator()(const std::pair<T1, T2> &p) const
    {
        auto h1 = std::hash<T1>{}(p.first);  // вычисляем хэш первого элемента пары
        auto h2 = std::hash<T2>{}(p.second); // вычисляем хэш второго элемента пары
        return h1 ^ (h2 << 1);               // возвращаем комбинированный хэш с использованием побитового XOR и сдвига
    }
};

// структура для представления графа
struct Graph
{
    std::unordered_map<std::pair<double, double>, std::unique_ptr<Node>, pair_hash> nodes; // хранилище узлов графа

    // метод для поиска ближайшего узла к заданным координатам (широта, долгота)
    Node *find_closest_node(double lat, double lon)
    {
        double min_distance = std::numeric_limits<double>::max(); // инициализация минимального расстояния
        Node *node_founded = nullptr;                             // указатель на найденный узел

        // проход по всем узлам графа
        for (const auto &node : nodes)
        {
            // вычисление евклидова расстояния до текущего узла
            double distance = sqrt(pow(node.second->lat - lat, 2) + pow(node.second->lon - lon, 2));
            if (distance < min_distance)
            {                                     // если найденное расстояние меньше минимального
                node_founded = node.second.get(); // обновление указателя на найденный узел
                min_distance = distance;          // обновление минимального расстояния
            }
        }

        return node_founded; // возвращение указателя на ближайший узел
    }

    // метод для добавления нового узла в граф
    void add_node(double lon, double lat)
    {
        // проверка, существует ли узел с такими координатами
        if (nodes.find({lat, lon}) == nodes.end())
        {
            auto node = std::make_unique<Node>(); // создание уникального указателя на новый узел
            node->lon = lon;                      // установка долготы узла
            node->lat = lat;                      // установка широты узла
            nodes[{lat, lon}] = move(node);       // добавление узла в хранилище графа
        }
    }

    // метод для добавления ребра между двумя узлами
    void add_edge(double lon1, double lat1, double lon2, double lat2, double distance)
    {
        auto it1 = nodes.find({lat1, lon1}); // поиск первого узла по координатам
        auto it2 = nodes.find({lat2, lon2}); // поиск второго узла по координатам

        if (it1 != nodes.end() && it2 != nodes.end())
        {                                               // если оба узла найдены
            Node *node1 = it1->second.get();            // получение указателя на первый узел
            Node *node2 = it2->second.get();            // получение указателя на второй узел
            node1->edges.emplace_back(node2, distance); // добавление второго узла в список рёбер первого узла
            node2->edges.emplace_back(node1, distance); // добавление первого узла в список рёбер второго узла (двустороннее ребро)
        }
        else
        {
            std::cerr << "Ошибка: ";
            if (it1 == nodes.end())
            {
                std::cerr << "Узел 1 (" << lon1 << ", " << lat1 << ") не найден. ";
            }
            if (it2 == nodes.end())
            {
                std::cerr << "Узел 2 (" << lon2 << ", " << lat2 << ") не найден.";
            }
            std::cerr << std::endl;
        }
    }

    // метод для загрузки графа из файла
    void load_graph_from_file(const std::string &filename)
    {
        std::ifstream file(filename); // открытие файла для чтения
        if (!file.is_open())          // проверка, открылся ли файл
        {
            std::cerr << "Невозможно открыть файл: " << filename << std::endl;
            return;
        }

        std::string line; // переменная для хранения текущей строки

        while (std::getline(file, line))
        {                                         // чтение файла построчно
            std::stringstream ss(line);           // создание потока из строки
            std::string node_part, neighbor_part; // переменные для хранения части строки с узлом и соседями

            getline(ss, node_part, ':'); // получение части строки до двоеточия (узел)
            double lon1, lat1;
            if (sscanf(node_part.c_str(), "%lf,%lf", &lon1, &lat1) != 2)
            {
                std::cerr << "Не удалось прочитть координаты узла: " << node_part << std::endl;
                continue;
            }
            add_node(lon1, lat1); // добавление узла в граф

            while (getline(ss, neighbor_part, ';'))
            { // чтение соседей, разделенных точкой с запятой
                double lon2, lat2, distance;
                if (sscanf(neighbor_part.c_str(), "%lf,%lf,%lf", &lon2, &lat2, &distance) != 3)
                {
                    std::cerr << "Не удалось прочитать координаты соседа и расстояние: " << neighbor_part << std::endl;
                    continue;
                }
                add_node(lon2, lat2);                       // добавление соседа в граф
                add_edge(lon1, lat1, lon2, lat2, distance); // добавление ребра между узлом и соседом
            }
        }
    }

    std::vector<Node *> bfs(Node *start, Node *finish)
    {
        std::queue<Node *> q;                      // очередь для хранения узлов для обхода
        std::unordered_map<Node *, Node *> parent; // хранилище для отслеживания родительских узлов
        std::unordered_set<Node *> visited;        // множество посещенных узлов
        std::vector<Node *> path;                  // вектор для хранения найденного пути

        q.push(start);           // добавление стартового узла в очередь
        visited.insert(start);   // пометка стартового узла как посещенного
        parent[start] = nullptr; // установка родителя стартового узла как nullptr

        while (!q.empty())
        {                              // пока очередь не пуста
            Node *current = q.front(); // получение текущего узла из очереди
            q.pop();                   // удаление текущего узла из очереди

            if (current == finish)
            { // если достигнут конечный узел
                while (current)
                {
                    path.push_back(current);   // добавление текущего узла в путь
                    current = parent[current]; // переход к родительскому узлу
                }
                reverse(path.begin(), path.end()); // обратный порядок пути (от конечного к стартовому)
                return path;                       // возвращение найденного пути
            }

            for (auto [neighbor, _] : current->edges)
            { // проход по всем соседям текущего узла
                if (visited.find(neighbor) == visited.end())
                {                               // если сосед еще не посещен
                    visited.insert(neighbor);   // пометка соседа как посещенного
                    parent[neighbor] = current; // установка текущего узла как родителя соседа
                    q.push(neighbor);           // добавление соседа в очередь для дальнейшего обхода
                }
            }
        }
        return path; // возвращение пустого пути, если конечный узел не найден
    }

    std::vector<Node *> dfs(Node *start, Node *finish)
    {
        std::stack<Node *> s;                      // создаем стек для хранения узлов во время обхода графа
        std::unordered_map<Node *, Node *> parent; // хранилище для отслеживания родительских узлов (для восстановления пути)
        std::unordered_set<Node *> visited;        // множество для хранения посещенных узлов
        std::vector<Node *> path;                  // вектор для хранения найденного пути

        s.push(start);           // добавляем стартовый узел в стек для начала обхода
        visited.insert(start);   // помечаем стартовый узел как посещенный
        parent[start] = nullptr; // устанавливаем родителя стартового узла как nullptr (нет родителя)

        while (!s.empty())
        {                            // пока стек не пустой
            Node *current = s.top(); // получаем текущий узел из вершины стека
            s.pop();                 // удаляем текущий узел из стека

            if (current == finish)
            { // если достигнут конечный узел
                while (current)
                {                              // восстанавливаем путь от конечного до стартового узла
                    path.push_back(current);   // добавляем текущий узел в путь
                    current = parent[current]; // переходим к родительскому узлу
                }
                reverse(path.begin(), path.end()); // оборачиваем путь, чтобы он был от стартового до конечного узла
                return path;                       // возвращаем найденный путь
            }

            for (auto [neighbor, _] : current->edges)
            { // проходим по всем соседям текущего узла
                if (visited.find(neighbor) == visited.end())
                {                               // если сосед еще не посещен
                    visited.insert(neighbor);   // помечаем соседа как посещенного
                    parent[neighbor] = current; // устанавливаем текущий узел как родителя соседа
                    s.push(neighbor);           // добавляем соседа в стек для дальнейшего обхода
                }
            }
        }
        return path; // возвращаем пустой путь, если конечный узел не найден
    }

    std::vector<Node *> dijkstra(Node *start, Node *finish)
    {
        std::priority_queue<std::pair<double, Node *>, std::vector<std::pair<double, Node *>>, std::greater<>> pq; // создаем приоритетную очередь для хранения узлов с их расстояниями
        std::unordered_map<Node *, double> distances;                                                              // хранилище для расстояний от стартового узла до каждого узла
        std::unordered_map<Node *, Node *> parent;                                                                 // хранилище для отслеживания родительских узлов

        for (auto &node : nodes)
        {
            distances[node.second.get()] = std::numeric_limits<double>::infinity(); // инициализируем расстояния до всех узлов как бесконечность
        }

        distances[start] = 0;    // расстояние до стартового узла равно 0
        pq.push({0, start});     // добавляем стартовый узел в очередь с расстоянием 0
        parent[start] = nullptr; // устанавливаем родителя стартового узла как nullptr

        while (!pq.empty())
        {
            auto [current_dist, current] = pq.top(); // получаем узел с наименьшим расстоянием из очереди
            pq.pop();                                // удаляем этот узел из очереди

            if (current_dist > distances[current])
                continue; // если текущее расстояние больше уже известного, пропускаем этот узел

            if (current == finish)
            {
                std::vector<Node *> path; // вектор для хранения найденного пути
                while (current)
                {
                    path.push_back(current);   // добавляем текущий узел в путь
                    current = parent[current]; // переходим к родительскому узлу
                }
                reverse(path.begin(), path.end()); // оборачиваем путь, чтобы он был от стартового до конечного узла
                return path;                       // возвращаем найденный путь
            }

            for (auto [neighbor, edge_dist] : current->edges)
            {
                if (edge_dist < 0)
                {
                    std::cerr << "Ошибка: отрицательное ребро" << std::endl;
                    continue; // пропускаем отрицательное ребро
                }
                double new_dist = current_dist + edge_dist; // вычисляем новое расстояние до соседа

                if (new_dist < distances[neighbor])
                {
                    distances[neighbor] = new_dist; // обновляем расстояние до соседа
                    pq.push({new_dist, neighbor});  // добавляем соседа в очередь с обновленным расстоянием
                    parent[neighbor] = current;     // устанавливаем текущий узел как родителя соседа
                }
            }
        }

        return {}; // возвращаем пустой вектор, если путь не найден
    }

    // вычисление расстояния между двумя узлами
    double heuristic(Node *a, Node *b)
    {
        return sqrt(pow(a->lat - b->lat, 2) + pow(a->lon - b->lon, 2)); // вычисляем эвристическую функцию
    }

    std::vector<Node *> a_star(Node *start, Node *finish)
    {
        std::priority_queue<std::pair<double, Node *>, std::vector<std::pair<double, Node *>>, std::greater<>> open_set; // приоритетная очередь для хранения открытых узлов с их оценками f(n)
        std::unordered_map<Node *, double> g_score;                                                                      // хранилище для g(n) - стоимости пути от стартового узла до каждого узла
        std::unordered_map<Node *, double> f_score;                                                                      // хранилище для f(n) - полной стоимости пути через узел (g(n) + h(n))
        std::unordered_map<Node *, Node *> came_from;                                                                    // хранилище для отслеживания родительских узлов

        // инициализируем g(n) и f(n) для всех узлов как бесконечность
        for (const auto &node : nodes)
        {
            g_score[node.second.get()] = std::numeric_limits<double>::infinity(); // инициализируем g(n) как бесконечность
            f_score[node.second.get()] = std::numeric_limits<double>::infinity(); // инициализируем f(n) как бесконечность
        }

        g_score[start] = 0;                        // стоимость пути от стартового узла до самого себя равна 0
        f_score[start] = heuristic(start, finish); // вычисляем начальное значение f(n) для стартового узла
        open_set.push({f_score[start], start});    // добавляем стартовый узел в открытый список с его оценкой f(n)
        came_from[start] = nullptr;                // устанавливаем родителя стартового узла как nullptr

        while (!open_set.empty())
        {
            Node *current = open_set.top().second; // получаем узел с наименьшей оценкой f(n)
            open_set.pop();                        // удаляем этот узел из открытого списка

            if (current == finish)
            {
                std::vector<Node *> path; // вектор для хранения найденного пути
                while (current)
                {
                    path.push_back(current);      // добавляем текущий узел в путь
                    current = came_from[current]; // переходим к родительскому узлу
                }
                reverse(path.begin(), path.end()); // разворачиваем путь, чтобы он был от стартового до конечного узла
                return path;                       // возвращаем найденный путь
            }

            for (auto [neighbor, weight] : current->edges) // проходим по всем соседям текущего узла
            {
                if (weight < 0)
                {
                    std::cerr << "Ошибка: отрицательное ребро" << std::endl;
                    continue; // пропускаем отрицательное ребро
                }
                
                double tentative_g_score = g_score[current] + weight; // вычисляем предполагаемую стоимость пути до соседа через текущий узел

                if (tentative_g_score < g_score[neighbor])
                {
                    came_from[neighbor] = current;                                       // устанавливаем текущий узел как родителя соседа
                    g_score[neighbor] = tentative_g_score;                               // обновляем g(n) для соседа
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, finish); // обновляем f(n) для соседа
                    open_set.push({f_score[neighbor], neighbor});                        // добавляем соседа в открытый список с обновленным значением f(n)
                }
            }
        }

        return {}; // возвращаем пустой вектор, если путь не найден
    }

    // вывод узлов и длины пути
    void print_path(std::vector<Node *> &path)
    {
        if (path.empty())
        {
            std::cout << "Путь не найден." << std::endl;
            return;
        }

        /* for (Node *n : path)
        {
            std::cout << n->lon << " " << n->lat << std::endl; // выводим координаты узла
        } */
        std::cout << path.size() << " узлов" << std::endl; // выводим длину пути
    }

    // подсчёт времени bfs
    void time_bfs(Node *start_node, Node *finish_node)
    {
        auto start_time = std::chrono::high_resolution_clock::now(); // фиксируем время начала
        std::vector<Node *> path = bfs(start_node, finish_node);     // вызываем bfs для поиска пути
        auto end_time = std::chrono::high_resolution_clock::now();   // фиксируем время окончания

        std::cout << "BFS: ";
        print_path(path);                                                                                                  // выводим найденный путь
        std::cout << "время BFS: " << std::chrono::duration<double>(end_time - start_time).count() << " сек" << std::endl; // выводим время выполнения
    }

    void time_dfs(Node *start_node, Node *finish_node)
    {
        auto start_time = std::chrono::high_resolution_clock::now(); // фиксируем время начала
        std::vector<Node *> path = dfs(start_node, finish_node);     // вызываем dfs для поиска пути
        auto end_time = std::chrono::high_resolution_clock::now();   // фиксируем время окончания

        std::cout << "DFS: ";
        print_path(path);                                                                                                  // выводим найденный путь
        std::cout << "время DFS: " << std::chrono::duration<double>(end_time - start_time).count() << " сек" << std::endl; // выводим время выполнения
    }

    void time_dijkstra(Node *start_node, Node *finish_node)
    {
        auto start_time = std::chrono::high_resolution_clock::now();  // фиксируем время начала
        std::vector<Node *> path = dijkstra(start_node, finish_node); // вызываем алгоритм Дейкстры для поиска пути
        auto end_time = std::chrono::high_resolution_clock::now();    // фиксируем время окончания

        std::cout << "Dijkstra: ";
        print_path(path);                                                                                                       // выводим найденный путь
        std::cout << "время Dijkstra: " << std::chrono::duration<double>(end_time - start_time).count() << " сек" << std::endl; // выводим время выполнения
    }

    void time_a_star(Node *start_node, Node *finish_node)
    {
        auto start_time = std::chrono::high_resolution_clock::now(); // фиксируем время начала
        std::vector<Node *> path = a_star(start_node, finish_node);  // вызываем A* для поиска пути
        auto end_time = std::chrono::high_resolution_clock::now();   // фиксируем время окончания

        std::cout << "A*: ";
        print_path(path);                                                                                                 // выводим найденный путь
        std::cout << "время A*: " << std::chrono::duration<double>(end_time - start_time).count() << " сек" << std::endl; // выводим время выполнения
    }
};

void run_spb_graph()
{
    Graph graph;                                 // создаем объект графа
    graph.load_graph_from_file("spb_graph.txt"); // загружаем граф из файла

    double start_lon = 30.48455, start_lat = 59.95878;                   // дом
    double finish_lon = 30.295483, finish_lat = 59.944077;               // биржа
    Node *start_node = graph.find_closest_node(start_lat, start_lon);    // находим ближайший узел к начальной точке
    Node *finish_node = graph.find_closest_node(finish_lat, finish_lon); // находим ближайший узел к конечной точке

    graph.time_bfs(start_node, finish_node); // подсчитываем время BFS
    std::cout << std::endl;

    graph.time_dfs(start_node, finish_node); // подсчитываем время DFS
    std::cout << std::endl;
    graph.time_dijkstra(start_node, finish_node); // подсчитываем время Dijkstra
    std::cout << std::endl;

    graph.time_a_star(start_node, finish_node); // подсчитываем время A*
    std::cout << std::endl;
};

int main()
{
    run_spb_graph(); // запускаем функцию для работы с графом
    return 0;        // завершаем программу
}
