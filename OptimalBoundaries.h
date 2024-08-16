// This file defines the OptimalBoundaries class,
// which is used to compute the *optimal boundaries for Poisson mesh merging*

#include <vector>

#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include "Mesh.h"

class OptimalBoundaries
{
public:
    OptimalBoundaries(const Mesh &source_mesh, const Mesh &target_mesh,
                      const std::vector<Mesh::VertexHandle> &source_boundary_vertices,
                      const std::vector<Mesh::VertexHandle> &feature_boundary_vertices,
                      const std::vector<Mesh::VertexHandle> &target_boundary_vertices);

    void preprocessing();

    void run();

    void set_para_bindings(double scale, double rotation, const Eigen::Vector2d &offset);

    Mesh &get_band() { return band; }
    Mesh &get_band_filled() { return band_filled; }
    Mesh &get_stripe() { return stripe; }
    Mesh &get_disk() { return disk; }

    Eigen::MatrixXd &get_band_uv() { return band_uv; }
    Eigen::MatrixXd &get_disk_uv() { return disk_uv; }

    auto &get_optimal_boundary() { return optimal_boundary; }

private:
    const Mesh &source_mesh;
    const Mesh &target_mesh;

    std::vector<Mesh::VertexHandle> optimal_boundary;

    // We assume that the boundary vertices are ordered in the right-hand rule (ccw) manner
    // (i.e. when you walk along the boundary, the mesh is on your left)
    const std::vector<Mesh::VertexHandle> &source_boundary_vertices;  // boundary of Omega_0
    const std::vector<Mesh::VertexHandle> &feature_boundary_vertices; // boundary of Omega_feature
    const std::vector<Mesh::VertexHandle> &target_boundary_vertices;  // boundary of Omega_1

    Mesh band;                                        // Omega_0 - Omega_feature
    OpenMesh::VProp<Mesh::VertexHandle> band_to_orig; // Mapping from band vertices to source mesh vertices
    OpenMesh::VProp<Mesh::VertexHandle> orig_to_band; // Mapping from source mesh vertices to band vertices

    Mesh stripe;                                        // the stripe cut from band (Omega_0 - Omega_feature)
    OpenMesh::VProp<Mesh::VertexHandle> stripe_to_band; // Mapping from stripe vertices to band vertices
    OpenMesh::VProp<Mesh::VertexHandle> band_to_stripe; // Mapping from band vertices to stripe vertices

    std::vector<Mesh::VertexHandle> seam_vertices_stripe_1;
    std::vector<Mesh::VertexHandle> seam_vertices_stripe_2;

    Mesh band_filled; // Omega_0 - Omega_feature with hole filled, for parameterization
    Eigen::MatrixXd band_uv;

    struct PointQuery
    {
        Mesh::FaceHandle face;
        double w0, w1, w2; // barycentric coordinates
        Eigen::Vector3d point;
    };

    OpenMesh::VProp<PointQuery> band_to_disk; // Query of each vertex in the band mesh

    Mesh disk;                                       // Omega_1 (on the target mesh)
    OpenMesh::VProp<Mesh::VertexHandle> disk_to_tgt; // Mapping from disk vertices to band vertices
    OpenMesh::VProp<Mesh::VertexHandle> tgt_to_disk; // Mapping from band vertices to disk vertices
    Eigen::MatrixXd disk_uv;

    // parameterization bindings
    double scale;
    double rotation; // rad
    Eigen::Vector2d offset;

    // Preprocessing
    // ==================================================

    // Generate the band mesh (Omega_0 - Omega_feature)
    void gen_band_mesh();

    // Cut the band mesh into a stripe (for shortest closed path tracing)
    void gen_stripe_mesh();

    // Fill in the hole in the band mesh for parameterization
    void gen_filled_mesh();

    // Cut out Omega_1 from the target mesh
    void gen_disk_mesh();

    // Parameterize the filled band
    void parameterize_source();

    // Parameterize the disk
    void parameterize_target();

    // Binding the band and the disk
    // ==================================================

    // Find the face in the target mesh that contains the point, in the UV domain
    PointQuery disk_rasterization(const Eigen::Vector2d &point);

    // Update the query of each vertex in the band mesh
    void update_band_to_disk();

    // Main algorithm
    // ==================================================
    void find_optimal_boundary();
};