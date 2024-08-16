#include <queue>

#include "Dijkstra.h"

Dijkstra::Dijkstra(const Mesh &mesh,
                   Mesh::VertexHandle source,
                   const OpenMesh::EProp<double> &edge_weight)
    : mesh(mesh), source(source), edge_weight(edge_weight),
      distance(std::numeric_limits<double>::infinity(), mesh),
      previous(Mesh::VertexHandle(), mesh) {}

Dijkstra::Dijkstra(const Mesh &mesh,
                   Mesh::VertexHandle source,
                   Mesh::VertexHandle target,
                   const OpenMesh::EProp<double> &edge_weight)
    : mesh(mesh), source(source), target(target), edge_weight(edge_weight),
      distance(std::numeric_limits<double>::infinity(), mesh),
      previous(Mesh::VertexHandle(), mesh) {}

void Dijkstra::run()
{
    distance[source] = 0.0;

    // Initialize the priority queue
    std::priority_queue<std::pair<double, Mesh::VertexHandle>,
                        std::vector<std::pair<double, Mesh::VertexHandle>>,
                        std::greater<>>
        queue;
    queue.push({0.0, source});

    // Run Dijkstra's algorithm
    while (!queue.empty())
    {
        auto [d, fr] = queue.top();
        queue.pop();

        if (d > distance[fr])
            continue;

        if (fr == target)
            break;

        for (const auto &he : mesh.voh_range(fr))
        {
            const auto to = he.to();
            const auto w = edge_weight[he.edge()];

            if (distance[to] > distance[fr] + w)
            {
                distance[to] = distance[fr] + w;
                previous[to] = fr;
                queue.push({distance[to], to});
            }
        }
    }
}
