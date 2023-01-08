#pragma once
#include "AutoMorphingModel.h"
#include "Scene.h"

#include <memory>
#include <utility>

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void MouseCallback(cg3d::Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[]) override;
    void ScrollCallback(cg3d::Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]) override;
    void CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState)  override;
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void print_arms_tip_position();
    void print_destination_position();
    void pick_next_link();
    void IK_CCD_SOLVER();
    void set_IK_solver_flag();
    bool arms_tip_reached_dest();
    void print_rotation_matrix();
    Eigen::Vector3f sphere_position();
    Eigen::Vector3f link_tip_position(int index);
    Eigen::Vector3f link_center_position(int index);
    Eigen::Vector3f link_source_position(int index);
    Eigen::Vector3f GetSpherePos();


private:
    Eigen::Vector3f dest_sphere_position;
    Eigen::Vector3f arm_tip_position;
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> sphere1 ,cube;
    std::shared_ptr<cg3d::AutoMorphingModel> autoCube;
    std::vector<std::shared_ptr<cg3d::Model>> cyls, axis;
    Eigen::Vector3f link_len;
    bool start_IK_solver = false;
    int pickedIndex = 0;
    int tipIndex = 0;
    float DISTANCE_DELTA;
    float ANGEL_STEPS;
    float temp_const1;
    float temp_const2;
    float angel_constant;

    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F,E,EF,EI;
    Eigen::VectorXi EQ;
  // If an edge were collapsed, we'd collapse it to these points:
    Eigen::MatrixXd V, C, N, T, points,edges,colors;
};
