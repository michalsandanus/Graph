#pragma once

#include <vector>
#include <optional>
#include <stdexcept>
#include <queue>
#include <algorithm>

// define 1 if you want to implement it
#define BONUS 1

// Edge can be constructed with syntax
// Edge e{ 1, 5, 1.89 };
struct Edge
{
    int v;
    int u;
    double weight;
};


inline bool operator>(const Edge& lhs, const Edge& rhs) {
    return lhs.weight > rhs.weight;
}

/* Graph represents one abstract graph.
 *
 * Vertices are always numbered from 0.
 *
 * Between two vertices,  there can only be one edge.   If edges is in
 * form of  (v, v), so  both ends are  the same, e.g.   AddEdge({1, 1,
 * 0.5}) throw an exception.
 *
 * Also throw excepton if one or both vertices are out of bounds.
 *
 * Graph is not directed so (u, v) is the same edge as (v, u).
 *
 * Weights are  limited to positive  values, so 0 and  negative values
 * are forbidden.  If you encounter such weights throw an exception.
 */

class Graph
{
    // Do not modify public interface
public:
    // Construct graph with n vertices and no edges
    explicit Graph(size_t n);

    /* Will construct graph with given  edges, the vertices are from 0
     * to whatever is the highest number in the vector of edges.
     */
    explicit Graph(const std::vector<Edge>& edges);

    /* Add  edge to  graph. If  the edge  already exists,  replace the
     * weight.
     */
    void AddEdge(const Edge& edge);

    /* Same as AddEdge, but can insert  multiple edges. If one edge is
     * there more  than once,  use later one  (edge with  higher index
     * overwrites edge with lower index)
     */
    void AddEdges(const std::vector<Edge>& edges);

    /* Return  weight between  vertices  u  and v.  If  edge does  not
     * exists, behaviour is undefined.
     */
    double operator()(int u, int v) const;

    /* Return weight between vertices u and v, if edge does not exists,
     * throw an exception.
     */
    double At(int u, int v) const;

    /* Return true  if there  is an  edge between  u and  v, otherwise
     * false.
     */
    bool Connected(int u, int v) const noexcept;

    /* Return shortest path  from u to v (path with  minimal cost). If
     * there is no path return nullopt.   You can use dijkstra, or any
     * other algorithm. Path should start with u and ends with v.
     *
     * https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
     *
     * 1 BONUS point will be for  speed of this function.
     * First 5 implementations will be awarded with the bonus point.
     */
    std::optional<std::vector<int>> Path(int u, int v) const;

#if BONUS == 1
    /* Returns  minimum spanning  tree  for this  graph.  You can  use
     * kruskal's algorithm
     * https://en.wikipedia.org/wiki/Kruskal%27s_algorithm
     */
    Graph SpannigTree() const;
#endif

private:
    // Here you can add whatever you want.
    std::vector<std::vector<double>> m_vertices;
    std::vector<bool> m_ver_present;

    bool AreInSet(int u, int v, std::vector<int> set) const {
        bool u_in = false;
        bool v_in = false;
        
        for (size_t i = 0; i < set.size(); i++) {
            if (set[i] == u) u_in = true;
            else if (set[i] == v) v_in = true;
        }

        return u_in && v_in;
    }

    int IsInSet(int u, std::vector<int> set) const {
        bool u_in = false;

        for (size_t i = 0; i < set.size(); i++) {
            if (set[i] == u) u_in = true;
        }

        return u_in;
    }
};

inline Graph::Graph(size_t n) {
    std::vector<std::vector<double>> v(n, std::vector<double>(n, 0));
    std::vector<bool> p(n, true);
    m_vertices = v;
    m_ver_present = p;
}

inline Graph::Graph(const std::vector<Edge>& edges) {
    int biggest_vertex = -1;
    for (size_t i = 0; i < edges.size(); i++) {
        if (edges[i].u > biggest_vertex) {
            biggest_vertex = edges[i].u;
        }

        if (edges[i].v > biggest_vertex) {
            biggest_vertex = edges[i].v;
        }
    }

    if (biggest_vertex == -1) {
        throw std::invalid_argument("Could not construct graph without edges!");
    }

    else {
        std::vector<std::vector<double>> v(biggest_vertex + 1, std::vector<double>(biggest_vertex + 1, 0));
        std::vector<bool> p(biggest_vertex + 1, false);
        m_vertices = v;
        m_ver_present = p;

        for (size_t i = 0; i < edges.size(); i++) {
            if (edges[i].u == edges[i].v) {
                throw std::invalid_argument("Could not construct graph with loop!");
                break;
            }

            if (edges[i].weight <= 0) {
                throw std::invalid_argument("Could not construct graph with not positive edge weight!");
                break;
            }

            m_vertices[edges[i].u][edges[i].v] = edges[i].weight;
            m_vertices[edges[i].v][edges[i].u] = edges[i].weight;
            m_ver_present[edges[i].u] = true;
            m_ver_present[edges[i].v] = true;
        }
    }
}

inline void Graph::AddEdge(const Edge& edge) {
    if (edge.weight <= 0) throw std::invalid_argument("Could not add edge with not positive weight!");
    if (edge.u == edge.v) throw std::invalid_argument("Could not add edge causing a loop in graph!");
    if (edge.u < 0 || edge.v < 0) throw std::invalid_argument("Could not add edge with negative vertex!");
    if (static_cast<size_t>(edge.u) >= m_ver_present.size() || static_cast<size_t>(edge.v) >= m_ver_present.size()) throw std::invalid_argument("Could not add edge to not existing vertex!");
    if (!m_ver_present[edge.u] || !m_ver_present[edge.u]) throw std::invalid_argument("Could not add edge to not existing vertex!");

    m_vertices[edge.u][edge.v] = edge.weight;
    m_vertices[edge.v][edge.u] = edge.weight;
}

inline void Graph::AddEdges(const std::vector<Edge>& edges) {
    for (size_t i = 0; i < edges.size(); i++) {
        if (edges[i].weight <= 0) throw std::invalid_argument("Could not add edge with not positive weight!");
        if (edges[i].u == edges[i].v) throw std::invalid_argument("Could not add edge causing a loop in graph!");
        if (edges[i].u < 0 || edges[i].v < 0) throw std::invalid_argument("Could not add edge with negative vertex!");
        if (static_cast<size_t>(edges[i].u) >= m_ver_present.size() || static_cast<size_t>(edges[i].v) >= m_ver_present.size()) throw std::invalid_argument("Could not add edge to not existing vertex!");
        if (!m_ver_present[edges[i].u] || !m_ver_present[edges[i].u]) throw std::invalid_argument("Could not add edge to not existing vertex!");
    }

    for (size_t i = 0; i < edges.size(); i++) {
        m_vertices[edges[i].u][edges[i].v] = edges[i].weight;
        m_vertices[edges[i].v][edges[i].u] = edges[i].weight;
    }
}

inline double Graph::operator()(int u, int v) const {
    return m_vertices[u][v];
}

inline  double Graph::At(int u, int v) const {
    if (u < 0 || v < 0) throw std::invalid_argument("Edge does not exist");
    if (static_cast<size_t>(u) >= m_ver_present.size() || static_cast<size_t>(v) >= m_ver_present.size()) throw std::invalid_argument("Edge does not exist");
    if (m_vertices[u][v] == 0) throw std::invalid_argument("Edge does not exist");

    return m_vertices[u][v];
}

inline bool Graph::Connected(int u, int v) const noexcept {
    if (u < 0 || v < 0) return false;
    if (static_cast<size_t>(u) >= m_ver_present.size() || static_cast<size_t>(v) >= m_ver_present.size()) return false;
    if (m_vertices[u][v] == 0) return false;

    return true;
}

inline std::optional<std::vector<int>> Graph::Path(int u, int v) const {
    std::vector<int> path;
    if (u == v) {
        path.push_back(u);
        return path;
    }

    struct Vertex {
        int u;
        double weight;
    };

    auto compare = [](Vertex lhs, Vertex rhs)
    {
        return lhs.weight > rhs.weight;
    };


    std::vector<int> previous_vertex(m_ver_present.size(), -1);
    std::vector<double> weight_to_vertex(m_ver_present.size(), -1);
    std::vector<bool> explored_vertices(m_ver_present.size(), false);

    weight_to_vertex[u] = 0;

    std::priority_queue<Vertex, std::vector<Vertex>, decltype(compare)> min_heap(compare);

    for (size_t i = 0; i < m_ver_present.size(); i++) {
        if (m_vertices[u][i] != 0 && m_ver_present[u]) {
            Vertex ver{ static_cast<int>(i), m_vertices[u][i] };

            weight_to_vertex[i] = m_vertices[u][i];
            previous_vertex[i] = u;

            min_heap.push(ver);
        }
    }

    explored_vertices[u] = true;


    while (!min_heap.empty()) {
        Vertex act_vertex = min_heap.top();
        min_heap.pop();

        if (weight_to_vertex[v] != -1 && act_vertex.weight >= weight_to_vertex[v]) break;

        //update weights
        for (size_t i = 0; i < m_ver_present.size(); i++) {
            if (m_vertices[act_vertex.u][i] != 0 && m_ver_present[act_vertex.u] && !explored_vertices[i]) {
                Vertex ver{ static_cast<int>(i), weight_to_vertex[act_vertex.u] + m_vertices[act_vertex.u][i] };

                if (weight_to_vertex[i] == -1 || weight_to_vertex[i] > weight_to_vertex[act_vertex.u] + m_vertices[act_vertex.u][i]) {
                    weight_to_vertex[i] = weight_to_vertex[act_vertex.u] + m_vertices[act_vertex.u][i];
                    previous_vertex[i] = act_vertex.u;

                    min_heap.push(ver);
                }
            }
        }

        explored_vertices[act_vertex.u] = true;
    }

    if (previous_vertex[v] == -1) return std::nullopt;
    else {
        if (u != v) {
            path.push_back(v);
            int prev = previous_vertex[v];

            while (prev != u) {
                path.push_back(prev);
                prev = previous_vertex[prev];
            }

            path.push_back(u);

            std::reverse(path.begin(), path.end());
        }
    }

    return path;
}

inline Graph Graph::SpannigTree() const {
    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> min_heap;
    std::vector<Edge> vec_edges;

    for (size_t i = 0; i < m_ver_present.size(); i++) {
        for (size_t j = 0; j <= i; j++) {
            if (m_vertices[i][j] != 0) {
                Edge edge{ static_cast<int>(i),static_cast<int>(j),m_vertices[i][j] };
                min_heap.push(edge);
            }
        }
    }

    std::vector<std::vector<int>> ver_sets;

    while (!min_heap.empty()) {
        Edge edge = min_heap.top();
        min_heap.pop();

        bool loop = false;
        for (size_t i = 0; i < ver_sets.size(); i++) {
            if (AreInSet(edge.u, edge.v, ver_sets[i])) {
                loop = true;
                break;
            }
        }

        if (!loop) {
            vec_edges.push_back(edge);

            int u_set = -1;
            int v_set = -1;
            for (size_t i = 0; i < ver_sets.size(); i++) {
                if (IsInSet(edge.u, ver_sets[i])) {
                    u_set = static_cast<int>(i);
                }

                if (IsInSet(edge.v, ver_sets[i])) {
                    v_set = static_cast<int>(i);
                }
            }

            if (u_set == -1 && v_set == -1) {
                std::vector<int> new_set;
                new_set.push_back(edge.u);
                new_set.push_back(edge.v);
                ver_sets.push_back(new_set);
            }

            else if (u_set == -1) {
                ver_sets[v_set].push_back(edge.u);
            }

            else if (v_set == -1) {
                ver_sets[u_set].push_back(edge.v);
            }

            else {
                std::vector<int> new_vec;
                new_vec.reserve(ver_sets[u_set].size() + ver_sets[v_set].size());
                new_vec.insert(new_vec.end(), ver_sets[u_set].begin(), ver_sets[u_set].end());
                new_vec.insert(new_vec.end(), ver_sets[v_set].begin(), ver_sets[v_set].end());

                if (u_set > v_set) {
                    ver_sets.erase(ver_sets.begin() + u_set);
                    ver_sets.erase(ver_sets.begin() + v_set);
                }

                else {
                    ver_sets.erase(ver_sets.begin() + v_set);
                    ver_sets.erase(ver_sets.begin() + u_set);
                }

                ver_sets.push_back(new_vec);
            }
        }
    }

    Graph spanning_tree(vec_edges);
    return spanning_tree;

}
