#include <Eigen/Sparse>

#include "Parameterization.h"

void Parameterization::pack()
{
    double min_u = uv.col(0).minCoeff(),
           max_u = uv.col(0).maxCoeff(),
           min_v = uv.col(1).minCoeff(),
           max_v = uv.col(1).maxCoeff();

    uv.col(0) = (uv.col(0).array() - min_u) / (max_u - min_u);
    uv.col(1) = (uv.col(1).array() - min_v) / (max_v - min_v);
}

double Tutte::opposite_angle(Mesh::HalfedgeHandle heh) const
{
    auto p0 = mesh.point(mesh.from_vertex_handle(heh));
    auto p1 = mesh.point(mesh.to_vertex_handle(heh));
    auto p2 = mesh.point(mesh.to_vertex_handle(mesh.next_halfedge_handle(heh)));

    return acos(((p1 - p2).normalized()).dot((p0 - p2).normalized()));
}

void Tutte::compute_cot()
{
    cotangents.resize(mesh.n_halfedges(), 0.0);

    for (auto heh : mesh.halfedges())
    {
        if (heh.is_boundary() || !heh.is_valid())
            continue;
        cotangents[heh.idx()] = 1.0 / tan(opposite_angle(heh));
    }
}

void Tutte::find_boundary_vertices()
{
    boundary_vertices.clear();

    Mesh::HalfedgeHandle first_boundary_he;

    for (auto heh : mesh.halfedges())
        if (mesh.is_boundary(heh))
        {
            first_boundary_he = heh;
            break;
        }

    // OpenMesh Doc: If you are on a boundary, the next halfedge is guaranteed to be also a boundary halfedge.
    for (auto heh = first_boundary_he; heh.is_valid(); heh = mesh.next_halfedge_handle(heh))
    {
        boundary_vertices.push_back(mesh.to_vertex_handle(heh));
        if (mesh.to_vertex_handle(heh) == mesh.from_vertex_handle(first_boundary_he))
            break;
    }
}

Eigen::SparseMatrix<double> Tutte::laplacian_uniform()
{
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(mesh.n_halfedges() + mesh.n_vertices());

    for (auto v : mesh.vertices())
    {
        double valence = mesh.valence(v);
        for (auto vv : mesh.vv_range(v))
            triplets.push_back({v.idx(), vv.idx(), -1.0 / valence});
        triplets.push_back({v.idx(), v.idx(), 1.0});
    }

    Eigen::SparseMatrix<double> L(mesh.n_vertices(), mesh.n_vertices());
    L.setFromTriplets(triplets.begin(), triplets.end());

    return L;
}

Eigen::SparseMatrix<double> Tutte::laplacian_cotangent()
{
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(mesh.n_halfedges() * 4);

    for (auto heh : mesh.halfedges())
    {
        if (heh.is_boundary() || !heh.is_valid())
            continue;

        double cot = cotangents[heh.idx()];
        triplets.push_back({heh.from().idx(), heh.to().idx(), cot});
        triplets.push_back({heh.to().idx(), heh.from().idx(), cot});
        triplets.push_back({heh.from().idx(), heh.from().idx(), -cot});
        triplets.push_back({heh.to().idx(), heh.to().idx(), -cot});
    }

    Eigen::SparseMatrix<double> L(mesh.n_vertices(), mesh.n_vertices());
    L.setFromTriplets(triplets.begin(), triplets.end());

    return L;
}

void Tutte::tutte()
{
    find_boundary_vertices();

    switch (laplacian_type)
    {
    case LaplacianType::UNIFORM:
        laplacian = laplacian_uniform();
        break;
    case LaplacianType::COTANGENT:
        compute_cot();
        laplacian = laplacian_cotangent();
        break;
    }

    int n_vertices = mesh.n_vertices();

    // Fix the boundary vertices to the unit circle
    Eigen::MatrixX2d b(n_vertices, 2);
    b.setZero();
    for (int i = 0; i < boundary_vertices.size(); i++)
    {
        int idx_vi = boundary_vertices[i].idx();
        b(idx_vi, 0) = cos(2 * M_PI * i / boundary_vertices.size());
        b(idx_vi, 1) = sin(2 * M_PI * i / boundary_vertices.size());
    }

    // Construct the matrix A for the linear system Ax = b
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(laplacian.nonZeros());

    // Add the boundary constraints
    for (int i = 0; i < boundary_vertices.size(); i++)
        triplets.push_back({boundary_vertices[i].idx(), boundary_vertices[i].idx(), 1.0});

    // Extract triplets from the laplacian matrix
    std::vector<int> boundaryVertexIndices(boundary_vertices.size());
    for (int i = 0; i < boundary_vertices.size(); i++)
        boundaryVertexIndices[i] = boundary_vertices[i].idx();
    std::sort(boundaryVertexIndices.begin(), boundaryVertexIndices.end());
    for (int k = 0; k < laplacian.outerSize(); ++k)
    {
        // Skip the rows of boundary vertices
        // Assume that the matrix is symmetric
        if (std::binary_search(boundaryVertexIndices.begin(), boundaryVertexIndices.end(), k))
            continue;

        for (Eigen::SparseMatrix<double>::InnerIterator it(laplacian, k); it; ++it)
        {
            // Add the triplet to the list
            if (laplacian.Flags & Eigen::RowMajorBit)
                triplets.push_back({(int)it.row(), (int)it.col(), it.value()}); // Row major
            else
                triplets.push_back({(int)it.col(), (int)it.row(), it.value()}); // Column major
        }
    }

    // Construct the sparse matrix A
    Eigen::SparseMatrix<double> A(n_vertices, n_vertices);
    A.setFromTriplets(triplets.begin(), triplets.end());

    // Solve the linear system
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);

    uv = solver.solve(b);
}

void Tutte::flatten()
{
    tutte();
    pack();
}

// Signed SVD decomposition of a 2x2 matrix
// (i.e. sigma_2 could be negative)
struct SignedSVD22
{
    // A = U * S * V^T
    Eigen::Matrix2d U;
    Eigen::Matrix2d V;
    Eigen::Matrix2d S;

    SignedSVD22(Eigen::Matrix2d &A)
    {
        double E = (A(0, 0) + A(1, 1)) / 2,
               F = (A(0, 0) - A(1, 1)) / 2,
               G = (A(1, 0) + A(0, 1)) / 2,
               H = (A(1, 0) - A(0, 1)) / 2;
        double Q = sqrt(E * E + H * H), R = sqrt(F * F + G * G);
        double S1 = Q + R, S2 = Q - R;
        double T1 = atan2(G, F), T2 = atan2(H, E);
        double theta = (T2 - T1) / 2, phi = (T2 + T1) / 2;

        U = Eigen::Rotation2Dd(phi).toRotationMatrix();
        V = Eigen::Rotation2Dd(theta).toRotationMatrix().transpose();
        S << S1, 0.0, 0.0, S2;
    }

    Eigen::Matrix2d best_fit_ARAP()
    {
        return U * V.transpose();
    }

    Eigen::Matrix2d best_fit_ASAP()
    {
        double s = (S(0, 0) + S(1, 1)) / 2;
        Eigen::Matrix2d mean_S;
        mean_S << s, 0.0, 0.0, s;
        return U * mean_S * V.transpose();
    }
};

void LocalGlobal::local_global()
{
    // Initial guess
    tutte(); // also computes the cotangents and the laplacian

    // Precompute the solver
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(laplacian);

    // Directly flatten the faces to the 2D plane
    std::vector<Eigen::Matrix2d> triangleXs(mesh.n_faces());
    for (auto f : mesh.faces())
    {
        auto heh = mesh.halfedge_handle(f);
        auto v0 = mesh.from_vertex_handle(heh),
             v1 = mesh.to_vertex_handle(heh),
             v2 = mesh.to_vertex_handle(mesh.next_halfedge_handle(heh));

        auto x0 = mesh.point(v0),
             x1 = mesh.point(v1),
             x2 = mesh.point(v2);

        Eigen::Vector2d e01, e02;
        double theta = acos((x1 - x0).normalized().dot((x2 - x0).normalized()));
        e01 << (x1 - x0).norm(), 0;
        e02 = (x2 - x0).norm() * Eigen::Vector2d(cos(theta), sin(theta));

        // Use row vectors
        triangleXs[f.idx()] << e01(0), e01(1),
            e02(0), e02(1);
    }

    for (int iter = 1; iter <= num_iter; iter++)
    {
        // Local step
        Eigen::MatrixX2d rhs(mesh.n_vertices(), 2);
        rhs.setZero();

        for (auto f : mesh.faces())
        {
            // Get the 2D triangle in the UV plane, from the last iteration
            auto heh01 = mesh.halfedge_handle(f);
            auto v0 = mesh.from_vertex_handle(heh01),
                 v1 = mesh.to_vertex_handle(heh01),
                 v2 = mesh.to_vertex_handle(mesh.next_halfedge_handle(heh01));

            Eigen::Vector2d u0 = uv.row(v0.idx()), u1 = uv.row(v1.idx()), u2 = uv.row(v2.idx());

            auto e01 = u1 - u0, e02 = u2 - u0;

            Eigen::Matrix2d triangleU;
            triangleU << e01(0), e01(1),
                e02(0), e02(1);

            // Compute the best fit rotation from the Jacobi matrix
            // x and u are row vectors, so x * J == u, J = x^-1 * u
            Eigen::Matrix2d J = triangleXs[f.idx()].inverse() * triangleU;

            Eigen::Matrix2d R;
            switch (target)
            {
            case LocalGlobalTarget::ARAP:
                R = SignedSVD22(J).best_fit_ARAP();
                break;
            case LocalGlobalTarget::ASAP:
                R = SignedSVD22(J).best_fit_ASAP();
                break;
            }

            // Compute the right hand side of the linear system
            Eigen::Matrix<double, 1, 2> x0, x1, x2;
            x0 << 0, 0;
            x1 << triangleXs[f.idx()](0, 0), triangleXs[f.idx()](0, 1);
            x2 << triangleXs[f.idx()](1, 0), triangleXs[f.idx()](1, 1);

            auto heh12 = mesh.next_halfedge_handle(heh01);
            auto heh20 = mesh.next_halfedge_handle(heh12);

            rhs.block<1, 2>(v0.idx(), 0) += cotangents[heh01.idx()] * ((x0 - x1) * R) + cotangents[heh20.idx()] * ((x0 - x2) * R);
            rhs.block<1, 2>(v1.idx(), 0) += cotangents[heh12.idx()] * ((x1 - x2) * R) + cotangents[heh01.idx()] * ((x1 - x0) * R);
            rhs.block<1, 2>(v2.idx(), 0) += cotangents[heh20.idx()] * ((x2 - x0) * R) + cotangents[heh12.idx()] * ((x2 - x1) * R);
        }

        // Global step
        uv = solver.solve(rhs);
    }
}

void LocalGlobal::flatten()
{
    local_global();
    pack();
}