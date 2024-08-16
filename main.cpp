#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include "Mesh.h"
#include "Dijkstra.h"
#include "OptimalBoundaries.h"

#include "MyGL/Window.h"
#include "MyGL/Mesh.h"
#include "MyGL/PointCloud.h"
#include "MyGL/Shader.h"
#include "MyGL/PickVertex.h"

#include <iostream>

glm::vec3 eigen_to_glm_vec3d(Eigen::Vector3d v)
{
    return {v.x(), v.y(), v.z()};
}

struct MeshConverter
{
    MeshConverter(const Mesh &mesh) : mesh(mesh)
    {
        center = glm::vec3(0.0f);
        vertices.reserve(mesh.n_vertices());
        for (const auto &vertex : mesh.vertices())
        {
            vertices.push_back({mesh.point(vertex).x(),
                                mesh.point(vertex).y(),
                                mesh.point(vertex).z()});
            center += vertices.back();
        }
        center /= static_cast<float>(mesh.n_vertices());

        indices.reserve(mesh.n_faces() * 3);
        for (const auto &face : mesh.faces())
            for (const auto &vertex : mesh.fv_range(face))
                indices.push_back(vertex.idx());

        MyGL::Mesh gl_mesh(vertices, indices);
    }

    const Mesh &mesh;

    glm::vec3 center;
    std::vector<glm::vec3> vertices;
    std::vector<GLuint> indices;
};

class ImGuiLog
{
public:
    void log(const std::string &entry)
    {
        log_entries.push_back(entry);
    }

    void show_log_window()
    {
        ImGui::Begin("Log");

        for (const auto &entry : log_entries)
            ImGui::Text("%s", entry.c_str());

        ImGui::End();
    }

private:
    std::vector<std::string> log_entries;
};

class SelectCut
{
public:
    SelectCut(const Mesh &mesh) : mesh(mesh), edge_length(mesh), is_selected(false, mesh)
    {
        for (const auto &e : mesh.edges())
            edge_length[e] = mesh.calc_edge_length(e);
    }

    bool add_vertex(Mesh::VertexHandle vertex)
    {
        if (!vertex.is_valid() || is_selected[vertex])
            return false;

        // If the seam is empty, add the vertex directly
        if (seam_vertices.empty())
        {
            is_selected[vertex] = true;
            seam_vertices.push_back(vertex);
            seam_points.push_back(eigen_to_glm_vec3d(mesh.point(vertex)));
            return true;
        }

        // Run Dijkstra's algorithm to find the shortest path from the last vertex to the new vertex
        auto start = seam_vertices.back();
        Dijkstra dijkstra(mesh, start, vertex, edge_length);
        dijkstra.run();
        if (dijkstra.get_distance(vertex) == std::numeric_limits<double>::infinity())
            return false;

        // Check if the path intersects with the existing seam
        for (Mesh::VertexHandle v = vertex; v != start; v = dijkstra.get_previous(v))
            if (is_selected[v])
                return false;

        int num_seam_old = seam_vertices.size();
        for (Mesh::VertexHandle v = vertex; v != start; v = dijkstra.get_previous(v))
            seam_vertices.push_back(v);
        std::reverse(seam_vertices.begin() + num_seam_old, seam_vertices.end());

        for (auto v = seam_vertices.begin() + num_seam_old; v != seam_vertices.end(); ++v)
        {
            is_selected[*v] = true;
            seam_points.push_back(eigen_to_glm_vec3d(mesh.point(*v)));
        }

        return true;
    }

    bool close_seam()
    {
        if (seam_vertices.empty())
            return false;

        auto start = seam_vertices.back();
        auto end = seam_vertices.front();
        Dijkstra dijkstra(mesh, start, end, edge_length);
        dijkstra.run();
        if (dijkstra.get_distance(end) == std::numeric_limits<double>::infinity() ||
            dijkstra.get_previous(end) == seam_vertices[1])
            return false;

        int num_seam_old = seam_vertices.size();
        for (Mesh::VertexHandle v = end; v != start; v = dijkstra.get_previous(v))
            seam_vertices.push_back(v);
        std::reverse(seam_vertices.begin() + num_seam_old, seam_vertices.end());
        seam_vertices.resize(seam_vertices.size() - 1); // remove the redundant end vertex

        for (auto v = seam_vertices.begin() + num_seam_old; v != seam_vertices.end(); ++v)
            seam_points.push_back(eigen_to_glm_vec3d(mesh.point(*v)));

        is_closed = true;

        return true;
    }

    bool is_seam_closed() const
    {
        return is_closed;
    }

    std::string vertices_to_string() const
    {
        std::string result;
        for (const auto &v : seam_vertices)
            result += std::to_string(v.idx()) + " ";
        return result;
    }

    std::vector<Mesh::VertexHandle> seam_vertices;
    std::vector<glm::vec3> seam_points;

private:
    const Mesh &mesh;
    OpenMesh::EProp<double> edge_length;
    OpenMesh::VProp<bool> is_selected;

    bool is_closed = false;
};

int main()
{
    // Initialize window (and OpenGL context)
    MyGL::Window window(1500, 1000, "Cross Field Visualization");
    float last_frame_time = 0.0f;
    float delta_time = 0.0f;

    // Load shader from file
    MyGL::ShaderProgram basic;
    try
    {
        basic.load_from_file("data/shaders/basic.vert", "data/shaders/basic.frag");
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }

    // Load mesh from file
    Mesh source_mesh, target_mesh;
    try
    {
        OpenMesh::IO::read_mesh(source_mesh, "data/models/max-planck.obj");
        OpenMesh::IO::read_mesh(target_mesh, "data/models/ball.obj");
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }

    // Resize the mesh to [-1, 1]^3
    auto resize_mesh = [](Mesh &mesh)
    {
        Eigen::Vector3d min_point(std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::infinity());
        Eigen::Vector3d max_point(-std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity());
        for (const auto &v : mesh.vertices())
        {
            auto point = mesh.point(v);
            for (int i = 0; i < 3; i++)
            {
                min_point[i] = std::min(min_point[i], point[i]);
                max_point[i] = std::max(max_point[i], point[i]);
            }
        }

        Eigen::Vector3d center = (min_point + max_point) / 2.0;
        Eigen::Vector3d scale = max_point - min_point;
        double max_scale = scale.maxCoeff();
        for (auto &v : mesh.vertices())
        {
            auto point = mesh.point(v);
            for (int i = 0; i < 3; i++)
                point[i] = (point[i] - center[i]) / max_scale;
            mesh.set_point(v, point);
        }
    };
    resize_mesh(source_mesh);
    resize_mesh(target_mesh);

    // Convert mesh to MyGL::Mesh
    MeshConverter src_conv(source_mesh), tgt_conv(target_mesh);

    MyGL::Mesh gl_mesh_src(src_conv.vertices, src_conv.indices);
    MyGL::Mesh gl_mesh_tgt(tgt_conv.vertices, tgt_conv.indices);

    // Set up camera
    MyGL::OrbitCamera camera(src_conv.center);
    camera.set_position({0.0f, 0.0f, 1.0f});

    // Some utility classes
    MyGL::PickVertex pick_vertex;

    ImGuiLog log;

    auto io = ImGui::GetIO();

    // Some status variables
    struct
    {
        bool wireframe_on = true;
        bool select_vertex_mode = false;
        bool load_boundary_from_file = false;
    } status;

    // Variables for selecting the boundaries
    SelectCut cut_src(source_mesh),
        cut_feat(source_mesh),
        cut_tgt(target_mesh);

    MyGL::PointCloud gl_pointcloud_s0(cut_src.seam_points),
        gl_pointcloud_feat(cut_feat.seam_points),
        gl_pointcloud_tgt(cut_tgt.seam_points);

    // GUI variables
    enum class MeshType
    {
        SOURCE,
        TARGET,
        RESULT
    } mesh_type = MeshType::SOURCE;

    enum class BoundaryType
    {
        SOURCE,
        FEATURE
    } boundary_type = BoundaryType::SOURCE;

    // Main loop 1 (selecting boundaries)
    // ==================================================
    while (!window.should_close())
    {
        if (status.load_boundary_from_file)
            break;

        // per-frame time logic
        float current_frame_time = static_cast<float>(glfwGetTime());
        delta_time = current_frame_time - last_frame_time;
        last_frame_time = current_frame_time;

        // input
        // ==================================================
        window.poll_events();
        window.process_input();
        window.process_input_camera(camera, delta_time);

        // update mvp matrices
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = camera.get_view_matrix() * camera.get_model_matrix();
        auto [width, height] = window.get_framebuffer_size();
        glm::mat4 projection = camera.get_projection_matrix(static_cast<float>(width) / height);

        basic.set_MVP(model, view, projection);

        // render
        // ==================================================
        glEnable(GL_MULTISAMPLE);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // draw the mesh
        basic.use();

        if (mesh_type == MeshType::SOURCE) // draw the source mesh
        {
            // mesh
            if (status.wireframe_on)
            {
                basic.set_uniform("color", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
                gl_mesh_src.draw_wireframe();
            }
            basic.set_uniform("color", glm::vec4(1.0f, 0.5f, 0.2f, 1.0f));
            gl_mesh_src.draw();

            // boundary of sigma_0
            glPointSize(10.0f);
            basic.set_uniform("color", glm::vec4(0.1f, 0.8f, 0.1f, 1.0f));
            gl_pointcloud_s0.draw();

            // boundary of sigma_feat
            basic.set_uniform("color", glm::vec4(0.1f, 0.1f, 0.8f, 1.0f));
            gl_pointcloud_feat.draw();
        }
        else if (mesh_type == MeshType::TARGET) // draw the target mesh
        {
            // mesh
            if (status.wireframe_on)
            {
                basic.set_uniform("color", glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
                gl_mesh_tgt.draw_wireframe();
            }
            basic.set_uniform("color", glm::vec4(1.0f, 0.5f, 0.2f, 1.0f));
            gl_mesh_tgt.draw();

            // boundary of sigma_tgt
            glPointSize(10.0f);
            basic.set_uniform("color", glm::vec4(0.8f, 0.1f, 0.1f, 1.0f));
            gl_pointcloud_tgt.draw();
        }

        // imgui
        // ==================================================
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Settings");

        ImGui::Checkbox("Wireframe", &status.wireframe_on);

        // Select which mesh to display
        const char *mesh_items[] = {"Source", "Target"};
        const char *current_mesh_item = mesh_items[static_cast<int>(mesh_type)];

        if (ImGui::BeginCombo("Mesh", current_mesh_item))
        {
            for (int n = 0; n < IM_ARRAYSIZE(mesh_items); n++)
            {
                bool is_selected = (current_mesh_item == mesh_items[n]);
                if (ImGui::Selectable(mesh_items[n], is_selected))
                {
                    current_mesh_item = mesh_items[n];
                    mesh_type = static_cast<MeshType>(n);
                }
                if (is_selected)
                    ImGui::SetItemDefaultFocus(); // Focus on the selected item
            }
            ImGui::EndCombo();
        }

        auto gui_add_vertex = [&](SelectCut &boundary, MyGL::Mesh &gl_mesh, MyGL::PointCloud &gl_pointcloud, glm::mat4 mvp)
        {
            if (boundary.is_seam_closed())
                ImGui::Text("The cut is closed");
            else
            {
                ImGui::Checkbox("Add vertex", &status.select_vertex_mode);
                if (ImGui::Button("Close cut"))
                    if (boundary.close_seam())
                    {
                        status.select_vertex_mode = false;
                        gl_pointcloud.update();
                    }
            }
            ImGui::Text(boundary.vertices_to_string().c_str());

            if (status.select_vertex_mode)
                if (!io.WantCaptureMouse && ImGui::IsMouseClicked(0)) // 0 is the left mouse button
                {
                    ImVec2 mouse_pos = ImGui::GetMousePos();
                    log.log("Mouse clicked at (" + std::to_string(mouse_pos.x) + ", " + std::to_string(mouse_pos.y) + ")");

                    auto [width, height] = window.get_framebuffer_size();
                    int index = pick_vertex.pick_vertex(mouse_pos.x, height - mouse_pos.y, gl_mesh, mvp);
                    log.log("Selected vertex " + std::to_string(index));

                    if (index >= 0) // -1 means no vertex is selected
                    {
                        boundary.add_vertex(source_mesh.vertex_handle(index));
                        gl_pointcloud.update();
                    }
                }
        };

        if (mesh_type == MeshType::SOURCE)
        {
            // Select which boundary to add vertices to
            const char *boundary_items[] = {"Source", "Feature"};
            const char *current_boundary_item = boundary_items[static_cast<int>(boundary_type)];

            ImGui::Text("Boundary of ");
            ImGui::SameLine();
            if (ImGui::BeginCombo("##combo", current_boundary_item)) // hide the label
            {
                for (int n = 0; n < IM_ARRAYSIZE(boundary_items); n++)
                {
                    bool is_selected = (current_boundary_item == boundary_items[n]);
                    if (ImGui::Selectable(boundary_items[n], is_selected))
                    {
                        current_boundary_item = boundary_items[n];
                        boundary_type = static_cast<BoundaryType>(n);
                    }
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            switch (boundary_type)
            {
            case BoundaryType::SOURCE:
                gui_add_vertex(cut_src, gl_mesh_src, gl_pointcloud_s0, projection * view * model);
                break;
            case BoundaryType::FEATURE:
                gui_add_vertex(cut_feat, gl_mesh_src, gl_pointcloud_feat, projection * view * model);
                break;
            }
        }
        else if (mesh_type == MeshType::TARGET)
            gui_add_vertex(cut_tgt, gl_mesh_tgt, gl_pointcloud_tgt, projection * view * model);

        if (cut_src.is_seam_closed() && cut_feat.is_seam_closed() && cut_tgt.is_seam_closed())
            if (ImGui::Button("Generate Optimal Boundaries"))
                break;

        ImGui::End();

        log.show_log_window();

        ImGui::Render();

        // render imgui and swap buffers
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        window.swap_buffers();
    }

    // if (!status.load_boundary_from_file)
    // {
    //     // print the selected vertices to a file
    //     std::ofstream seam_file("data/seam_vertices.txt");
    //     for (const auto &v : cut_src.seam_vertices)
    //         seam_file << v.idx() << " ";
    //     seam_file << std::endl;
    //     for (const auto &v : cut_feat.seam_vertices)
    //         seam_file << v.idx() << " ";
    //     seam_file << std::endl;
    //     for (const auto &v : cut_tgt.seam_vertices)
    //         seam_file << v.idx() << " ";
    //     seam_file << std::endl;
    // }

    std::vector<Mesh::VertexHandle> src_seam, feat_seam, tgt_seam;
    if (status.load_boundary_from_file)
    {
        // load the selected vertices from a file
        std::ifstream seam_file_in("data/seam_vertices.txt");
        auto read_a_line = [&](std::vector<Mesh::VertexHandle> &vertices)
        {
            std::string line;
            std::getline(seam_file_in, line);
            std::istringstream iss(line);
            int v_idx;
            while (iss >> v_idx)
                vertices.push_back(source_mesh.vertex_handle(v_idx));
        };
        read_a_line(src_seam);
        read_a_line(feat_seam);
        read_a_line(tgt_seam);
    }

    if (status.load_boundary_from_file)
    {
        cut_src.seam_vertices = src_seam;
        cut_feat.seam_vertices = feat_seam;
        cut_tgt.seam_vertices = tgt_seam;
    }

    OptimalBoundaries ob(source_mesh, target_mesh,
                         cut_src.seam_vertices, cut_feat.seam_vertices, cut_tgt.seam_vertices);
    ob.preprocessing();

    // OpenMesh::IO::write_mesh(ob.get_band(), "data/models/band.obj");
    // OpenMesh::IO::write_mesh(ob.get_band_filled(), "data/models/band_filled.obj");
    // OpenMesh::IO::write_mesh(ob.get_stripe(), "data/models/stripe.obj");

    auto gen_uv_mesh = [&](const Mesh &mesh, Eigen::MatrixXd &uv)
    {
        auto conv = MeshConverter(mesh);
        std::vector<glm::vec3> verticecs_uv(mesh.n_vertices());
        for (const auto &v : mesh.vertices())
            verticecs_uv[v.idx()] = {uv(v.idx(), 0), uv(v.idx(), 1), 0.0f};
        return MyGL::Mesh(verticecs_uv, conv.indices);
    };

    Mesh band = ob.get_band();
    auto band_uv = ob.get_band_uv();
    MyGL::Mesh gl_band = gen_uv_mesh(band, band_uv);

    Mesh disk = ob.get_disk();
    auto disk_uv = ob.get_disk_uv();
    MyGL::Mesh gl_disk = gen_uv_mesh(disk, disk_uv);

    struct
    {
        float scale = 0.7;
        float rotation_angle = 0.0;
        float offset_x = 0.0;
        float offset_y = 0.0;
    } para_binding;

    bool should_close = false;

    // Main loop 2 (bind parameterization)
    // ==================================================
    while (!window.should_close())
    {
        if (should_close)
            break;

        // input
        // ==================================================
        window.poll_events();
        window.process_input();

        // update mvp matrices
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::mat4(1.0f);
        auto [width, height] = window.get_framebuffer_size();
        glm::mat4 projection;
        float aspect_ratio = static_cast<float>(width) / static_cast<float>(height);
        if (aspect_ratio > 1.0f)
            projection = glm::ortho(-aspect_ratio, aspect_ratio, -1.0f, 1.0f, -1.0f, 1.0f);
        else
            projection = glm::ortho(-1.0f, 1.0f, -1.0f / aspect_ratio, 1.0f / aspect_ratio, -1.0f, 1.0f);

        basic.set_MVP(model, view, projection);

        // render
        // ==================================================
        glEnable(GL_MULTISAMPLE);
        glClearColor(1.0f, 1.0f, 1.0f, 0.2f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        basic.use();

        // draw the disk (from the target mesh)
        if (status.wireframe_on)
        {
            basic.set_uniform("color", glm::vec4(0.5f, 0.95f, 0.5f, 0.95f));
            gl_disk.draw_wireframe();
        }
        basic.set_uniform("color", glm::vec4(0.5f, 0.2f, 1.0f, 0.2f));
        gl_disk.draw();

        // draw the band (from the source mesh)
        model = glm::scale(model, glm::vec3(para_binding.scale));
        model = glm::rotate(model, para_binding.rotation_angle, glm::vec3(0.0f, 0.0f, 1.0f));
        model = glm::translate(model, glm::vec3(para_binding.offset_x, para_binding.offset_y, 0.1f));
        basic.set_uniform("model", model);

        if (status.wireframe_on)
        {
            basic.set_uniform("color", glm::vec4(0.95f, 0.95f, 0.95f, 0.95f));
            gl_band.draw_wireframe();
        }
        basic.set_uniform("color", glm::vec4(1.0f, 0.5f, 0.2f, 0.5f));
        gl_band.draw();

        // imgui
        // ==================================================
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Settings");

        ImGui::Checkbox("Wireframe", &status.wireframe_on);

        ImGui::SliderFloat("Scale", &para_binding.scale, 0.1f, 2.0f);
        ImGui::SliderAngle("Rotation", &para_binding.rotation_angle);
        ImGui::SliderFloat("Offset X", &para_binding.offset_x, -1.0f, 1.0f);
        ImGui::SliderFloat("Offset Y", &para_binding.offset_y, -1.0f, 1.0f);

        if (ImGui::Button("Confirm Binding"))
            should_close = true;

        ImGui::End();

        log.show_log_window();

        ImGui::Render();

        // render imgui and swap buffers
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        window.swap_buffers();
    }

    // Set the bindings
    ob.set_para_bindings(para_binding.scale, para_binding.rotation_angle / 180.0 * M_PI,
                         {para_binding.offset_x, para_binding.offset_y});

    mesh_type = MeshType::SOURCE;

    ob.run();

    auto optimal_boundary = ob.get_optimal_boundary();
    std::vector<glm::vec3> optimal_boundary_points(optimal_boundary.size());
    for (int i = 0; i < optimal_boundary.size(); i++)
        optimal_boundary_points[i] = glm::vec3(band_uv(optimal_boundary[i].idx(), 0),
                                               band_uv(optimal_boundary[i].idx(), 1),
                                               0.1f);
    MyGL::PointCloud gl_optimal_boundary(optimal_boundary_points);

    should_close = false;

    // Main loop 3
    // ==================================================
    while (!window.should_close())
    {
        if (should_close)
            break;

        // input
        // ==================================================
        window.poll_events();
        window.process_input();

        // update mvp matrices
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::mat4(1.0f);
        auto [width, height] = window.get_framebuffer_size();
        glm::mat4 projection;
        float aspect_ratio = static_cast<float>(width) / static_cast<float>(height);
        if (aspect_ratio > 1.0f)
            projection = glm::ortho(-aspect_ratio, aspect_ratio, -1.0f, 1.0f, -1.0f, 1.0f);
        else
            projection = glm::ortho(-1.0f, 1.0f, -1.0f / aspect_ratio, 1.0f / aspect_ratio, -1.0f, 1.0f);

        basic.set_MVP(model, view, projection);

        // render
        // ==================================================
        glEnable(GL_MULTISAMPLE);
        glClearColor(1.0f, 1.0f, 1.0f, 0.2f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        basic.use();

        // draw the disk (from the target mesh)
        if (status.wireframe_on)
        {
            basic.set_uniform("color", glm::vec4(0.5f, 0.95f, 0.5f, 0.95f));
            gl_disk.draw_wireframe();
        }
        basic.set_uniform("color", glm::vec4(0.5f, 0.2f, 1.0f, 0.2f));
        gl_disk.draw();

        // draw the band (from the source mesh)
        model = glm::scale(model, glm::vec3(para_binding.scale));
        model = glm::rotate(model, para_binding.rotation_angle, glm::vec3(0.0f, 0.0f, 1.0f));
        model = glm::translate(model, glm::vec3(para_binding.offset_x, para_binding.offset_y, 0.1f));
        basic.set_uniform("model", model);

        if (status.wireframe_on)
        {
            basic.set_uniform("color", glm::vec4(0.95f, 0.95f, 0.95f, 0.95f));
            gl_band.draw_wireframe();
        }
        basic.set_uniform("color", glm::vec4(1.0f, 0.5f, 0.2f, 0.5f));
        gl_band.draw();

        glPointSize(10.0f);
        basic.set_uniform("color", glm::vec4(0.1f, 0.8f, 0.1f, 1.0f));
        gl_optimal_boundary.draw();

        // imgui
        // ==================================================
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Settings");

        ImGui::Checkbox("Wireframe", &status.wireframe_on);

        ImGui::End();

        // log.show_log_window();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // render imgui and swap buffers
        window.swap_buffers();
    }

    return 0;
}