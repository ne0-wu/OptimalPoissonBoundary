#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include "Mesh.h"

class Parameterization
{
public:
    Parameterization(Mesh const &mesh) : mesh(mesh) {}
    virtual void flatten() = 0;
    Eigen::MatrixX2d get_uv() { return uv; }

protected:
    Mesh const &mesh;    // input mesh
    Eigen::MatrixX2d uv; // output uv coordinates

    void pack(); // Pack uv coordinates into [0, 1] x [0, 1]
};

class Tutte : public Parameterization
{
public:
    enum class LaplacianType
    {
        UNIFORM,
        COTANGENT
    };

    Tutte(Mesh const &mesh, LaplacianType laplacian_type = LaplacianType::UNIFORM)
        : Parameterization(mesh), laplacian_type(laplacian_type) {}

    void flatten();

protected:
    // Reusable variables
    std::vector<double> cotangents;
    std::vector<Mesh::VertexHandle> boundary_vertices;

    // Tutte parameterization
    void tutte(Eigen::SparseMatrix<double> const &laplacian);

    LaplacianType laplacian_type;
    Eigen::SparseMatrix<double> laplacian_uniform();
    Eigen::SparseMatrix<double> laplacian_cotangent();

    Eigen::SparseMatrix<double> laplacian;

    // Opposite angle of a halfedge
    double opposite_angle(Mesh::HalfedgeHandle heh) const;

    // Find boundary vertices
    void find_boundary_vertices();

    // Pre-compute cotangent
    void compute_cot();

    // Tutte's algorithm from 'How to Draw a Graph'
    void tutte();
};

class LocalGlobal : public Tutte
{
public:
    enum class LocalGlobalTarget
    {
        ARAP,
        ASAP
    };

    LocalGlobal(Mesh const &mesh, LocalGlobalTarget target = LocalGlobalTarget::ARAP, int num_iter = 100)
        : Tutte(mesh, LaplacianType::COTANGENT), target(target), num_iter(num_iter) {}

    void set_num_iter(int num_iter) { this->num_iter = num_iter; }

    void flatten();

protected:
    int num_iter;
    LocalGlobalTarget target;

    // Algorithm from 'A Local/Global Approach to Mesh Parameterization'
    void local_global();
};
