#include <queue>

#include "Dijkstra.h"
#include "Parameterization.h"

#include "OptimalBoundaries.h"

inline constexpr double infd = std::numeric_limits<double>::infinity();

OptimalBoundaries::OptimalBoundaries(const Mesh &source_mesh, const Mesh &target_mesh,
                                     const std::vector<Mesh::VertexHandle> &source_boundary_vertices,
                                     const std::vector<Mesh::VertexHandle> &feature_boundary_vertices,
                                     const std::vector<Mesh::VertexHandle> &target_boundary_vertices)
    : source_mesh(source_mesh),
      target_mesh(target_mesh),
      source_boundary_vertices(source_boundary_vertices),
      feature_boundary_vertices(feature_boundary_vertices),
      target_boundary_vertices(target_boundary_vertices),
      band_to_orig(band), orig_to_band(source_mesh),
      stripe_to_band(stripe), band_to_stripe(band),
      disk_to_tgt(disk), tgt_to_disk(target_mesh),
      band_to_disk(band) {}

void OptimalBoundaries::preprocessing()
{
    gen_band_mesh();
    gen_stripe_mesh();
    gen_filled_mesh();
    gen_disk_mesh();

    parameterize_source();
    parameterize_target();
}

void OptimalBoundaries::run()
{
    update_band_to_disk();

    find_optimal_boundary();
}

void OptimalBoundaries::set_para_bindings(double scale, double rotation, const Eigen::Vector2d &offset)
{
    this->scale = scale;
    this->rotation = rotation;
    this->offset = offset;
}

void add_vertex(Mesh::VertexHandle v, const Mesh &old_mesh, Mesh &new_mesh,
                OpenMesh::VProp<bool> &is_inserted,
                OpenMesh::VProp<Mesh::VertexHandle> &old_to_new,
                OpenMesh::VProp<Mesh::VertexHandle> &new_to_old)
{
    if (!is_inserted[v])
    {
        is_inserted[v] = true;
        auto new_v = new_mesh.add_vertex(old_mesh.point(v));
        old_to_new[v] = new_v;
        new_to_old[new_v] = v;
    }
};

void add_face(Mesh::FaceHandle f, const Mesh &old_mesh, Mesh &new_mesh,
              OpenMesh::FProp<bool> &is_inserted,
              OpenMesh::VProp<Mesh::VertexHandle> &old_to_new)
{
    if (!is_inserted[f])
    {
        is_inserted[f] = true;
        std::vector<Mesh::VertexHandle> vertices;
        for (const auto &v : old_mesh.fv_ccw_range(f))
            vertices.push_back(old_to_new[v]);
        new_mesh.add_face(vertices[0], vertices[1], vertices[2]);
    }
}

void OptimalBoundaries::gen_band_mesh()
{
    // Iterate through vertices in the band with a modified breadth-first search
    std::queue<Mesh::VertexHandle> queue;
    OpenMesh::VProp<bool> is_visited(false, source_mesh);
    OpenMesh::VProp<bool> is_inserted_vert(false, source_mesh);
    OpenMesh::FProp<bool> is_inserted_face(false, source_mesh);

    auto add_v = [&](Mesh::VertexHandle v)
    {
        add_vertex(v, source_mesh, band, is_inserted_vert, orig_to_band, band_to_orig);
    };
    auto add_f = [&](Mesh::FaceHandle f)
    {
        add_face(f, source_mesh, band, is_inserted_face, orig_to_band);
    };

    // Add the boundary faces to the band,
    // and push the opposite vertices to the queue.
    auto deal_with_boundary = [&](std::vector<Mesh::VertexHandle> boundary_vertices)
    {
        for (const auto &v : boundary_vertices)
        {
            is_visited[v] = true;
            add_v(v);
        }
        for (int i = 0; i < boundary_vertices.size(); i++)
        {
            int j = (i + 1) % boundary_vertices.size();

            Mesh::VertexHandle fr = boundary_vertices[i];
            Mesh::VertexHandle to = boundary_vertices[j];
            auto he = source_mesh.find_halfedge(fr, to);
            auto op = he.next().to();

            add_v(op);
            add_f(he.face());

            if (!is_visited[op])
            {
                queue.push(op);
                is_visited[op] = true;
            }
        }
    };
    deal_with_boundary(source_boundary_vertices);
    deal_with_boundary(feature_boundary_vertices);

    // Iterate through the rest of the band
    while (!queue.empty())
    {
        Mesh::VertexHandle v = queue.front();
        queue.pop();

        // Add adjacent vertices to the band, and push unvisited vertices to the queue
        for (const auto &v_a : source_mesh.vv_range(v))
        {
            add_v(v_a);
            if (!is_visited[v_a])
            {
                queue.push(v_a);
                is_visited[v_a] = true;
            }
        }

        // Add adjacent faces to the band
        for (const auto &f : source_mesh.vf_range(v))
            add_f(f);
    }
}

void OptimalBoundaries::gen_stripe_mesh()
{
    // Find a cut from the source boundary to the feature boundary
    auto cut_begin = orig_to_band(source_boundary_vertices[0]);
    OpenMesh::EProp<double> edge_length(band);
    for (const auto &e : band.edges())
        edge_length[e] = band.calc_edge_length(e);

    Dijkstra dijkstra(band, cut_begin, edge_length);
    dijkstra.run();

    double min_feat2src_distance = infd;
    Mesh::VertexHandle min_feat2src_vertex;
    for (const auto &v : feature_boundary_vertices)
    {
        double d = dijkstra.get_distance(orig_to_band[v]);
        if (d < min_feat2src_distance)
        {
            min_feat2src_distance = d;
            min_feat2src_vertex = orig_to_band[v];
        }
    }

    std::vector<Mesh::VertexHandle> seam_vertices;
    for (auto v = min_feat2src_vertex; v != cut_begin; v = dijkstra.get_previous(v))
        seam_vertices.push_back(v);
    seam_vertices.push_back(cut_begin);

    // Shorten the seam by removing the part of the seam on the boundary
    OpenMesh::VProp<bool> is_band_boundary(false, band);
    for (const auto &v : source_boundary_vertices)
        is_band_boundary[orig_to_band[v]] = true;
    for (const auto &v : feature_boundary_vertices)
        is_band_boundary[orig_to_band[v]] = true;

    auto new_begin = seam_vertices.begin(), new_end = seam_vertices.end();
    while (is_band_boundary[*(new_begin + 1)])
        new_begin++;
    while (is_band_boundary[*(new_end - 2)])
        new_end--;

    seam_vertices = std::vector<Mesh::VertexHandle>(new_begin, new_end);

    seam_vertices_stripe_1.reserve(seam_vertices.size());
    seam_vertices_stripe_2.reserve(seam_vertices.size());

    // Cut the band into a stripe
    // Iterate through vertices in the band with a modified breadth-first search
    std::queue<Mesh::VertexHandle> queue;
    OpenMesh::VProp<bool> is_visited(false, band);
    OpenMesh::VProp<bool> is_inserted_vert(false, band);
    OpenMesh::FProp<bool> is_inserted_face(false, band);

    auto add_v = [&](Mesh::VertexHandle v)
    {
        add_vertex(v, band, stripe, is_inserted_vert, band_to_stripe, stripe_to_band);
    };
    auto add_f = [&](Mesh::FaceHandle f)
    {
        add_face(f, band, stripe, is_inserted_face, band_to_stripe);
    };

    // Add the boundary faces to the band,
    // and push the opposite vertices to the queue.
    auto deal_with_seam = [&]()
    {
        // do it twice, because the seam vertices should be duplicated in the stripe
        for (const auto &v : seam_vertices)
        {
            auto new_v = stripe.add_vertex(band.point(v));
            stripe_to_band[new_v] = v;
            band_to_stripe[v] = new_v;
        }

        for (int i = 0; i < seam_vertices.size() - 2; i++)
        {
            Mesh::VertexHandle fr = seam_vertices[i];
            Mesh::VertexHandle to = seam_vertices[i + 1];
            Mesh::VertexHandle nx = seam_vertices[i + 2];

            is_visited[fr] = true;
            is_visited[to] = true;

            auto add_face_from_he = [&](OpenMesh::SmartHalfedgeHandle he)
            {
                auto f = he.face();
                for (const auto &v : f.vertices())
                {
                    add_v(v);
                    if (!is_visited[v])
                    {
                        queue.push(v);
                        is_visited[v] = true;
                    }
                }

                add_f(f);
            };

            // add the faces in the clockwise order
            auto he_start = band.find_halfedge(fr, to);
            auto he_end = band.find_halfedge(to, nx);

            for (auto he = he_start; he != he_end.opp(); he = he.next().opp())
                add_face_from_he(he);

            // if it is the first iteration, do it counter-clockwisely as well
            if (i == 0)
                for (auto he = he_start; !he.is_boundary(); he = he.prev().opp())
                    add_face_from_he(he);
        }
    };
    deal_with_seam();
    for (auto &v : seam_vertices)
        seam_vertices_stripe_1.push_back(band_to_stripe[v]);
    std::reverse(seam_vertices.begin(), seam_vertices.end());
    deal_with_seam();
    std::reverse(seam_vertices.begin(), seam_vertices.end());
    for (auto &v : seam_vertices)
        seam_vertices_stripe_2.push_back(band_to_stripe[v]);

    // Iterate through the rest of the stripe
    while (!queue.empty())
    {
        Mesh::VertexHandle v = queue.front();
        queue.pop();

        // Add adjacent vertices to the band, and push unvisited vertices to the queue
        for (const auto &v_a : band.vv_range(v))
        {
            add_v(v_a);
            if (!is_visited[v_a])
            {
                queue.push(v_a);
                is_visited[v_a] = true;
            }
        }

        // Add adjacent faces to the band
        for (const auto &f : band.vf_range(v))
            add_f(f);
    }
}

template <typename T>
class vector_2d
{
public:
    vector_2d(int rows, int cols, T initial_value = 0)
        : rows(rows), cols(cols), data(rows * cols, initial_value) {}

    // Access the element at (i, j)
    // i: row index, j: column index
    T &operator()(int i, int j)
    {
        return data[i * cols + j];
    }

private:
    std::vector<T> data;
    int rows, cols;
};

void OptimalBoundaries::gen_filled_mesh()
{
    band_filled = band;

    // Find the boundary vertices of the hole to fill in
    // When you walk along the boundary, the band mesh is on your left and the hole is on your right
    std::vector<Mesh::VertexHandle> boundary_vertices = feature_boundary_vertices;
    std::reverse(boundary_vertices.begin(), boundary_vertices.end());
    for (auto &v : boundary_vertices)
        v = orig_to_band[v];

    // Fill in the hole using the method from 'Filling gaps in the boundary of a polyhedron'
    // Minimizing the total face area
    const int n = boundary_vertices.size();
    vector_2d<double> W(n, n, infd); // DP table for the total face area, W_{i,k} = min_{m} W_{i,m} + W_{m,k} + area_{i,m,k}
    vector_2d<int> O(n, n);          // the index m where the minimum is achieved

    auto triangle_area = [&](int a, int b, int c)
    {
        auto pa = band.point(boundary_vertices[a]);
        auto pb = band.point(boundary_vertices[b]);
        auto pc = band.point(boundary_vertices[c]);

        return 0.5 * Eigen::cross((pb - pa), (pc - pa)).norm();
    };

    // j = 2
    for (int i = 0; i < n; i++)
    {
        int m = (i + 1) % n;
        int k = (i + 2) % n;
        W(i, k) = triangle_area(i, m, k);
        O(i, k) = m;
    }

    // j > 2
    for (int j = 3; j <= n - 1; j++)
        for (int i = 0; i < n; i++)
        {
            int k_iv = i + j;                           // k (iterative variable)
            for (int m_iv = i + 1; m_iv < k_iv; m_iv++) // m (iterative variable)
            {
                int m = m_iv % n;
                int k = k_iv % n;
                if (W(i, k) > W(i, m) + W(m, k) + triangle_area(i, m, k))
                {
                    W(i, k) = W(i, m) + W(m, k) + triangle_area(i, m, k);
                    O(i, k) = m;
                }
            }
        }

    // Recursively fill in the hole using a queue
    double min_area = infd;
    int min_area_start = 0;
    for (int i = 0; i < n; i++)
    {
        int k = (i + n - 1) % n;
        if (W(i, k) < min_area)
        {
            min_area = W(i, k);
            min_area_start = i;
        }
    }

    std::queue<std::tuple<int, int, int>> queue;
    queue.push({min_area_start,
                O(min_area_start, (min_area_start + n - 1) % n),
                (min_area_start + n - 1) % n});
    auto add_f = [&](int i, int m, int k)
    {
        band_filled.add_face(boundary_vertices[i], boundary_vertices[m], boundary_vertices[k]);
    };
    while (!queue.empty())
    {
        auto [i, m, k] = queue.front();
        queue.pop();

        add_f(i, m, k);

        if (m != (i + 1) % n)
            queue.push({i, O(i, m), m});
        if (k != (m + 1) % n)
            queue.push({m, O(m, k), k});
    }
}

void OptimalBoundaries::gen_disk_mesh()
{
    // Iterate through vertices in the disk with a modified breadth-first search
    std::queue<Mesh::VertexHandle> queue;
    OpenMesh::VProp<bool> is_visited(false, target_mesh);
    OpenMesh::VProp<bool> is_inserted_vert(false, target_mesh);
    OpenMesh::FProp<bool> is_inserted_face(false, target_mesh);

    auto add_v = [&](Mesh::VertexHandle v)
    {
        add_vertex(v, target_mesh, disk, is_inserted_vert, tgt_to_disk, disk_to_tgt);
    };
    auto add_f = [&](Mesh::FaceHandle f)
    {
        add_face(f, target_mesh, disk, is_inserted_face, tgt_to_disk);
    };

    // Boundary vertices and faces
    for (const auto &v : target_boundary_vertices)
    {
        is_visited[v] = true;
        add_v(v);
    }
    for (int i = 0; i < target_boundary_vertices.size(); i++)
    {
        int j = (i + 1) % target_boundary_vertices.size();

        Mesh::VertexHandle fr = target_boundary_vertices[i];
        Mesh::VertexHandle to = target_boundary_vertices[j];
        auto he = target_mesh.find_halfedge(fr, to);
        auto op = he.next().to();

        add_v(op);
        add_f(he.face());

        if (!is_visited[op])
        {
            queue.push(op);
            is_visited[op] = true;
        }
    }

    // Iterate through the rest of the disk
    while (!queue.empty())
    {
        Mesh::VertexHandle v = queue.front();
        queue.pop();

        // Add adjacent vertices to the disk, and push unvisited vertices to the queue
        for (const auto &v_a : target_mesh.vv_range(v))
        {
            add_v(v_a);
            if (!is_visited[v_a])
            {
                queue.push(v_a);
                is_visited[v_a] = true;
            }
        }

        // Add adjacent faces to the disk
        for (const auto &f : target_mesh.vf_range(v))
            add_f(f);
    }
}

void OptimalBoundaries::parameterize_source()
{
    LocalGlobal local_global(band_filled);
    local_global.flatten();
    band_uv = local_global.get_uv();
    band_uv = 2.0 * band_uv.array() - 1.0; // rescale to [-1, 1], for ease of scaling and rotation
}

void OptimalBoundaries::parameterize_target()
{
    LocalGlobal local_global(disk);
    local_global.flatten();
    disk_uv = local_global.get_uv();
    disk_uv = 2.0 * disk_uv.array() - 1.0;
}

OptimalBoundaries::PointQuery OptimalBoundaries::disk_rasterization(const Eigen::Vector2d &point)
{
    // Note that this is a rasterization process, so we can switch to OpenGL to do it more efficiently

    for (const auto &f : disk.faces())
    {
        auto v0 = f.halfedge().from();
        auto v1 = f.halfedge().to();
        auto v2 = f.halfedge().next().to();
        auto p0 = disk_uv.row(v0.idx());
        auto p1 = disk_uv.row(v1.idx());
        auto p2 = disk_uv.row(v2.idx());

        auto signed_area = [](const Eigen::Vector2d &a, const Eigen::Vector2d &b, const Eigen::Vector2d &c)
        {
            return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]);
        };

        double area = signed_area(p0, p1, p2);
        double w0 = signed_area(p1, p2, point) / area;
        double w1 = signed_area(p2, p0, point) / area;
        double w2 = signed_area(p0, p1, point) / area;

        if (w0 >= 0 && w1 >= 0 && w2 >= 0)
            return {f, w0, w1, w2,
                    w0 * disk.point(v0) + w1 * disk.point(v1) + w2 * disk.point(v2)};
    }

    return {Mesh::FaceHandle(), -1, -1, -1, Eigen::Vector3d::Zero()};
}

void OptimalBoundaries::update_band_to_disk()
{
    for (const auto &v : band.vertices())
    {
        auto uv = band_uv.row(v.idx());
        uv = offset + Eigen::Rotation2Dd(rotation).matrix() * scale * Eigen::Vector2d(uv[0], uv[1]);
        band_to_disk[v] = disk_rasterization(uv);
    }
}

void OptimalBoundaries::find_optimal_boundary()
{
    // Initialize the optimal boundary with \partial Sigma_0
    optimal_boundary = source_boundary_vertices;
    for (auto &v : optimal_boundary)
        v = orig_to_band[v];

    OpenMesh::EProp<double> band_edge_length(band);
    for (const auto &e : band.edges())
        band_edge_length[e] = band.calc_edge_length(e);

    for (int num_iter = 1; num_iter <= 100; num_iter++)
    {
        int n = optimal_boundary.size();

        // Get the point lists
        Eigen::MatrixXd band_points(n, 3), disk_points(n, 3);
        for (int i = 0; i < n; i++)
        {
            auto v = optimal_boundary[i];
            band_points.row(i) = band.point(v);
            disk_points.row(i) = band_to_disk[v].point;
        }

        // Calculate average edge length and rescale
        double band_edge_length_average = 0.0, disk_edge_length_average = 0.0;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            band_edge_length_average += (band_points.row(j) - band_points.row(i)).norm();
            disk_edge_length_average += (disk_points.row(j) - disk_points.row(i)).norm();
        }
        double s = disk_edge_length_average / band_edge_length_average;
        band_points *= s;

        // Calculate the centroid and move to origin to eliminate translation
        Eigen::Vector3d band_centroid = band_points.colwise().mean();
        Eigen::Vector3d disk_centroid = disk_points.colwise().mean();
        band_points.rowwise() -= band_centroid.transpose();
        disk_points.rowwise() -= disk_centroid.transpose();

        // Calculate the covariance matrix and the rotation matrix
        Eigen::Matrix3d H = band_points.transpose() * disk_points;
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

        // Calculate the translation
        Eigen::Vector3d t = disk_centroid - R * band_centroid;

        // Done: the transformation is y = s * R * x + t

        // Calculate the weights
        OpenMesh::EProp<double> band_weight(band);
        for (const auto &e : band.edges())
        {
            auto he = e.halfedge();
            auto fr = he.from(), to = he.to();

            Eigen::Vector3d band_edge = band.point(to) - band.point(fr);
            band_edge *= s;
            Eigen::Vector3d disk_edge = band_to_disk[to].point - band_to_disk[fr].point;

            band_weight[e] = (R * band_edge - disk_edge).norm() * band_edge_length[e];

            // std::cout << band_edge.transpose() << ' '
            //           << disk_edge.transpose() << ' ' << band_weight[e] << '\n';
        }

        // Move the weights to the stripe
        OpenMesh::EProp<double> stripe_weight(stripe);
        for (const auto &e : stripe.edges())
        {
            auto he = e.halfedge();
            auto fr = he.from(), to = he.to();
            auto band_he = band.find_halfedge(stripe_to_band[fr], stripe_to_band[to]);
            stripe_weight[e] = band_weight[band_he.edge()];
        }

        // Find the shortest closed path on the stripe
        double min_cost = infd;
        int min_index = -1;

        for (int i = 0; i < seam_vertices_stripe_1.size(); i++)
        {
            auto v_start = seam_vertices_stripe_1[i];
            auto v_target = seam_vertices_stripe_2[i];

            Dijkstra dijkstra(stripe, v_start, v_target, stripe_weight);
            dijkstra.run();

            double cost = dijkstra.get_distance(v_target);
            if (cost < min_cost)
            {
                min_cost = cost;
                min_index = i;
            }
        }

        // Extract the optimal boundary
        std::vector<Mesh::VertexHandle> new_optimal_boundary;
        auto v_start = seam_vertices_stripe_1[min_index];
        auto v_target = seam_vertices_stripe_2[min_index];

        Dijkstra dijkstra(stripe, v_start, v_target, stripe_weight);
        dijkstra.run();

        for (auto v = v_target; v != v_start; v = dijkstra.get_previous(v))
            new_optimal_boundary.push_back(stripe_to_band[v]);
        new_optimal_boundary.push_back(stripe_to_band[v_start]);

        optimal_boundary = new_optimal_boundary;
    }
}