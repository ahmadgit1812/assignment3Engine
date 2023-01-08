#include "BasicScene.h"
#include <Eigen/src/Core/Matrix.h>
#include <edges.h>
#include <memory>
#include <per_face_normals.h>
#include <read_triangle_mesh.h>
#include <utility>
#include <vector>
#include "GLFW/glfw3.h"
#include "Mesh.h"
#include "PickVisitor.h"
#include "Renderer.h"
#include "ObjLoader.h"
#include "IglMeshLoader.h"

#include "igl/per_vertex_normals.h"
#include "igl/per_face_normals.h"
#include "igl/unproject_onto_mesh.h"
#include "igl/edge_flaps.h"
#include "igl/loop.h"
#include "igl/upsample.h"
#include "igl/AABB.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/circulation.h"
#include "igl/edge_midpoints.h"
#include "igl/collapse_edge.h"
#include "igl/edge_collapse_is_valid.h"
#include "igl/write_triangle_mesh.h"

// #include "AutoMorphingModel.h"

using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create( "camera", fov, float(width) / height, near, far);
    
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")}; 
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

 
    auto program = std::make_shared<Program>("shaders/phongShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");
    
    auto material{ std::make_shared<Material>("material", program)}; // empty material
    auto material1{ std::make_shared<Material>("material", program1)}; // empty material
//    SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());
 
    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj")};
    auto cylMesh{IglLoader::MeshFromFiles("cyl_igl","data/zcylinder.obj")};
    auto cubeMesh{IglLoader::MeshFromFiles("cube_igl","data/cube_old.obj")};
    sphere1 = Model::Create( "sphere",sphereMesh, material);    
    cube = Model::Create( "cube", cubeMesh, material);
    
    //Axis
    Eigen::MatrixXd vertices(6,3);
    vertices << -1,0,0,1,0,0,0,-1,0,0,1,0,0,0,-1,0,0,1;
    Eigen::MatrixXi faces(3,2);
    faces << 0,1,2,3,4,5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6,3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6,2);
    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys",vertices,faces,vertexNormals,textureCoords);
    axis.push_back(Model::Create("axis",coordsys,material1));
    axis[0]->mode = 1;   
    axis[0]->Scale(4,Axis::XYZ);
    // axis[0]->lineWidth = 5;
    root->AddChild(axis[0]);
    float scaleFactor = 1; 
    cyls.push_back( Model::Create("cyl",cylMesh, material));
    cyls[0]->Scale(scaleFactor,Axis::Z);
    cyls[0]->SetCenter(Eigen::Vector3f(0,0,-0.8f*scaleFactor));
    root->AddChild(cyls[0]);

    //
    link_len = Eigen::Vector3f(0, 0, 0.8f);
    DISTANCE_DELTA = 0.05f;
    ANGEL_STEPS = 10.f;
    temp_const1 = 180.f;
    temp_const2=3.14f;
   
    for(int i = 1;i < 3; i++)
    { 
        cyls.push_back( Model::Create("cyl", cylMesh, material));
        cyls[i]->Scale(scaleFactor,Axis::Z);
        cyls[i]->Translate(1.6f*scaleFactor,Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0,0,-0.8f*scaleFactor));
        cyls[i-1]->AddChild(cyls[i]);

        std::shared_ptr<Model> new_axis = Model::Create("axis",coordsys,material1);
        axis.push_back(new_axis);
        new_axis->mode =1;
        new_axis->Scale(4,Axis::XYZ);
        cyls[i-1]->AddChild(new_axis);
        new_axis->Translate(0.8f*scaleFactor, Axis::Z);

    }
    root->Translate({0,0,0.8f*scaleFactor});

    Eigen::Vector3f root_rotation_vector = Eigen::Vector3f(-1,0,0);
    root->RotateByDegree(90,root_rotation_vector);

    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
      return model->meshIndex;//(model->GetMeshList())[0]->data.size()-1;
    };
    autoCube = AutoMorphingModel::Create(*cube, morphFunc);

  
    sphere1->showWireframe = true;
    autoCube->Translate({-6,0,0});
    autoCube->Scale(1.5f);
    sphere1->Translate({5,0,0});

    autoCube->showWireframe = true;
    camera->Translate(22, Axis::Z);
    root->AddChild(sphere1);
//    root->AddChild(cyl);
    root->AddChild(autoCube);

    // points = Eigen::MatrixXd::Ones(1,3);
    // edges = Eigen::MatrixXd::Ones(1,3);
    // colors = Eigen::MatrixXd::Ones(1,3);
    
    // cyl->AddOverlay({points,edges,colors},true);
    cube->mode =1   ; 
    auto mesh = cube->GetMeshList();

    //autoCube->AddOverlay(points,edges,colors);
    // mesh[0]->data.push_back({V,F,V,E});
    int num_collapsed;

  // Function to reset original mesh and data structures
    V = mesh[0]->data[0].vertices;
    F = mesh[0]->data[0].faces;
   // igl::read_triangle_mesh("data/cube.off",V,F);
    igl::edge_flaps(F,E,EMAP,EF,EI);
    std::cout<< "vertices: \n" << V <<std::endl;
    std::cout<< "faces: \n" << F <<std::endl;
    
    std::cout<< "edges: \n" << E.transpose() <<std::endl;
    std::cout<< "edges to faces: \n" << EF.transpose() <<std::endl;
    std::cout<< "faces to edges: \n "<< EMAP.transpose()<<std::endl;
    std::cout<< "edges indices: \n" << EI.transpose() <<std::endl;
    sphere1->Translate({0,0,0});
    autoCube->Translate({0,0,0});
    angel_constant = temp_const1/temp_const2;

}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    program.SetUniform1f("specular_exponent", 5.0f);
    program.SetUniform4f("light_position", 0.0, 15.0f, 0.0, 1.0f);
    cube->Rotate(0.1f, Axis::XYZ);

    if(start_IK_solver)
       IK_CCD_SOLVER();
}

void BasicScene::MouseCallback(Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event

    if (action == GLFW_PRESS) { // default mouse button press behavior
        PickVisitor visitor;
        visitor.Init();
        renderer->RenderViewportAtPos(x, y, &visitor); // pick using fixed colors hack
        auto modelAndDepth = visitor.PickAtPos(x, renderer->GetWindowHeight() - y);
        renderer->RenderViewportAtPos(x, y); // draw again to avoid flickering
        pickedModel = modelAndDepth.first ? std::dynamic_pointer_cast<Model>(modelAndDepth.first->shared_from_this()) : nullptr;
        pickedModelDepth = modelAndDepth.second;
        camera->GetRotation().transpose();
        xAtPress = x;
        yAtPress = y;

        // if (pickedModel)
        //     debug("found ", pickedModel->isPickable ? "pickable" : "non-pickable", " model at pos ", x, ", ", y, ": ",
        //           pickedModel->name, ", depth: ", pickedModelDepth);
        // else
        //     debug("found nothing at pos ", x, ", ", y);

        if (pickedModel && !pickedModel->isPickable)
            pickedModel = nullptr; // for non-pickable models we need only pickedModelDepth for mouse movement calculations later

        if (pickedModel)
            pickedToutAtPress = pickedModel->GetTout();
        else
            cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::ScrollCallback(Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    bool arm_link_picked = false;
    int prev_index=-1;
    auto system = camera->GetRotation().transpose();
    if (pickedModel) {
        for(int i=0; i<cyls.size(); i++) {
            if (pickedModel == cyls[i]) {
                arm_link_picked = true;
                prev_index = i;
            }
        }

        if(arm_link_picked)
            pickedModel=cyls[0];

        pickedModel->TranslateInSystem(system * pickedModel->GetRotation(), {0, 0, -float(yoffset)});
        pickedToutAtPress = pickedModel->GetTout();

        if(arm_link_picked)
            pickedModel=cyls[prev_index];
    } else {
        root->TranslateInSystem(system, {0, 0, -float(yoffset)});
    }
}

void BasicScene::CursorPosCallback(Viewport* viewport, int x, int y, bool dragging, int* buttonState)
{
    if (dragging) {
        auto system = camera->GetRotation().transpose() * GetRotation();
        auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
        auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
        if (pickedModel) {
            //pickedModel->SetTout(pickedToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE) {
                bool arm_link_picked = false;
                int prev_index=-1;

                for(int i=0; i<cyls.size(); i++) {
                    if (pickedModel == cyls[i]) {
                        arm_link_picked = true;
                        prev_index = i;
                    }
                }

                if(arm_link_picked)
                    pickedModel=cyls[0];

                pickedModel->TranslateInSystem(system * root->GetRotation(),
                                               {-float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0});

                if(arm_link_picked)
                    pickedModel=cyls[prev_index];
            }
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Y);
                pickedModel->RotateInSystem(system, float(yAtPress - y) / angleCoeff, Axis::X);
            }
        } else {
           // camera->SetTout(cameraToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                root->TranslateInSystem(system, {-float(xAtPress - x) / moveCoeff/10.0f, float( yAtPress - y) / moveCoeff/10.0f, 0});
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                root->RotateInSystem(system, float(x - xAtPress) / 180.0f, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                root->RotateInSystem(system, float(x - xAtPress) / angleCoeff, Axis::Y);
                root->RotateInSystem(system, float(y - yAtPress) / angleCoeff, Axis::X);
            }
        }
        xAtPress =  x;
        yAtPress =  y;
    }
}

void BasicScene::KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
            case GLFW_KEY_UP:
                cyls[pickedIndex]->RotateInSystem(system, 0.1f, Axis::X);
                break;
            case GLFW_KEY_DOWN:
                cyls[pickedIndex]->RotateInSystem(system, -0.1f, Axis::X);
                break;
            case GLFW_KEY_LEFT:
                cyls[pickedIndex]->RotateInSystem(system, 0.1f, Axis::Y);
                break;
            case GLFW_KEY_RIGHT:
                cyls[pickedIndex]->RotateInSystem(system, -0.1f, Axis::Y);
                break;
            case GLFW_KEY_W:
                camera->TranslateInSystem(system, {0, 0.1f, 0});
                break;
            case GLFW_KEY_S:
                camera->TranslateInSystem(system, {0, -0.1f, 0});
                break;
            case GLFW_KEY_A:
                camera->TranslateInSystem(system, {-0.1f, 0, 0});
                break;
            case GLFW_KEY_B:
                camera->TranslateInSystem(system, {0, 0, 0.1f});
                break;
            case GLFW_KEY_F:
                camera->TranslateInSystem(system, {0, 0, -0.1f});
                break;
            case GLFW_KEY_1:
                if( pickedIndex > 0)
                  pickedIndex--;
                break;
            case GLFW_KEY_2:
                if(pickedIndex < cyls.size()-1)
                    pickedIndex++;
                break;
            case GLFW_KEY_3:
                if( tipIndex >= 0)
                {
                  if(tipIndex == cyls.size())
                    tipIndex--;
                  sphere1->Translate(GetSpherePos());
                  tipIndex--;
                }
                break;
            case GLFW_KEY_4:
                if(tipIndex < cyls.size())
                {
                    if(tipIndex < 0)
                      tipIndex++;
                    sphere1->Translate(GetSpherePos());
                    tipIndex++;
                }
                break;
            case GLFW_KEY_T:
                print_arms_tip_position();
                break;
            case GLFW_KEY_D:
                print_destination_position();
                break;
            case GLFW_KEY_N:
                pick_next_link();
                break;
            case GLFW_KEY_SPACE:
                set_IK_solver_flag();
                break;
            case GLFW_KEY_P:
                print_rotation_matrix();
                break;
        }
    }
}

void BasicScene::print_rotation_matrix(){
    int picked_link_index=-1;
    for(int i=0; i<cyls.size();i++)
        if(pickedModel==cyls[i])
            picked_link_index=i;

    if(picked_link_index != -1){
        std::cout << "link "<< picked_link_index << " is picked." << std::endl;
        Eigen::Matrix3f picked_link_rot= cyls[picked_link_index]->GetRotation();

        std::cout << "link rotation: " << std::endl << picked_link_rot << std::endl;
        Eigen::Vector3f picked_link_angels = picked_link_rot.eulerAngles(2,0,2);

        std::cout << "link matrices: " << std::endl;
        Eigen::Matrix3f phi_matrix;
        phi_matrix << cos(picked_link_angels(0)), -sin(picked_link_angels(0)), 2
        , sin(picked_link_angels(0)), cos(picked_link_angels(0)), 0
        , 0, 0, 1;
        std::cout << "phi matrix: " << std::endl << phi_matrix << std::endl;


        Eigen::Matrix3f theta_matrix;
        theta_matrix << 1, 0, 0
                , 0, cos(picked_link_angels(1)), -sin(picked_link_angels(1))
                , 0, sin(picked_link_angels(1)), cos(picked_link_angels(1));
        std::cout << "theta matrix: " << std::endl << theta_matrix << std::endl;


        Eigen::Matrix3f psi_matrix;
        psi_matrix << cos(picked_link_angels(2)), -sin(picked_link_angels(2)), 0
                , sin(picked_link_angels(2)), cos(picked_link_angels(2)), 0
                , 0, 0, 1;
        std::cout << "psi matrix: " << std::endl << psi_matrix << std::endl;


        std::cout << "link angles: " << std::endl;
        std::cout << "phi: "<< picked_link_angels(0) * angel_constant<< std::endl;
        std::cout << "theta: "<< picked_link_angels(1) * angel_constant<< std::endl;
        std::cout << "psi: "<< picked_link_angels(2) * angel_constant<< std::endl;


    }else{
        std::cout << "scene rotation matrix: " << std::endl << root->GetRotation() << std::endl;
    }
}

void BasicScene::set_IK_solver_flag(){
    dest_sphere_position = sphere_position();
    Eigen::Vector3f arm_source_position = link_source_position(0);
    //distance between arm's source and dest center
    float arm_sphere_distance = (dest_sphere_position - arm_source_position).norm();
    if(1.6f * cyls.size() >= arm_sphere_distance)
        start_IK_solver = true;
    else
        std::cout << "cannot reach" << std::endl;
}

bool BasicScene::arms_tip_reached_dest(){
    arm_tip_position = link_tip_position(cyls.size()-1);
    //distance between arm's tip and dest center
    float arm_sphere_distance = (dest_sphere_position-arm_tip_position).norm();
    if(arm_sphere_distance < DISTANCE_DELTA){
        start_IK_solver=false;
        std::cout<<"destination reached, arm <-> sphere distance: "<<arm_sphere_distance<<std::endl;
        return true;
    }
    return false;
}

void BasicScene::IK_CCD_SOLVER(){
    for(int i = cyls.size()-1; i>=0; i--){

        if(arms_tip_reached_dest())
            return;

        Eigen::Vector3f curr_link_source_position = link_source_position(i);
        Eigen::Vector3f dest_source_vec = (dest_sphere_position - curr_link_source_position).normalized();
        Eigen::Vector3f tip_source_vec = (arm_tip_position - curr_link_source_position).normalized();
        float dot_product = dest_source_vec.dot(tip_source_vec);
        dot_product = dot_product > 1 ? 1 : dot_product < -1 ? -1 : dot_product ;
        float curr_angel = acosf(dot_product);
        curr_angel = curr_angel / ANGEL_STEPS;
        Eigen::Vector3f curr_plane_normal = tip_source_vec.cross(dest_source_vec);
        Eigen::Vector3f rot_vec = (cyls[i]->GetRotation().transpose() * curr_plane_normal).normalized();
        cyls[i]->Rotate(curr_angel, rot_vec);
    }
}

void BasicScene::pick_next_link(){
    int curr_picked_link_index = -1;
    for(int i=0; i<cyls.size(); i++)
        if(pickedModel == cyls[i])
            curr_picked_link_index=i;

    int next_link_index = curr_picked_link_index == -1 ? 0 : (curr_picked_link_index+1) % cyls.size();
    std::cout << "next link index is: " << next_link_index << std::endl;
    pickedModel = cyls[next_link_index];
}

void BasicScene::print_destination_position(){
    Eigen::Vector3f dest_sphere_position = sphere_position();
    std::cout << "destination position is: "<< "("<< dest_sphere_position(0) << ", "<<
    dest_sphere_position(1)<< ", " << dest_sphere_position(2) << ")" << std::endl;

}

Eigen::Vector3f BasicScene::sphere_position(){
    Eigen::MatrixXf sphereAggregatedTransform = sphere1->GetAggregatedTransform();
    return Eigen::Vector3f(sphereAggregatedTransform(0,3), sphereAggregatedTransform(1, 3), sphereAggregatedTransform(2,3));
}

void BasicScene::print_arms_tip_position(){
    for(int i=0; i<cyls.size(); i++) {
        Eigen::Vector3f curr_link_tip_position = link_tip_position(i);
        std::cout << "link " << i << " tip position is: " << "(" << curr_link_tip_position(0) << ", "
        << curr_link_tip_position(1) << ", "<< curr_link_tip_position(2) << ")" << std::endl;
    }
}

Eigen::Vector3f BasicScene::link_tip_position(int index){
    Eigen::Vector3f rotation_v = cyls[index]->GetRotation() * link_len;
    return link_center_position(index) + rotation_v;
}


Eigen::Vector3f BasicScene::link_center_position(int index){
    Eigen::Matrix4f linkAggregatedTransform = cyls[index]->GetAggregatedTransform();
    return Eigen::Vector3f(linkAggregatedTransform(0,3), linkAggregatedTransform(1,3), linkAggregatedTransform(2,3));
}
Eigen::Vector3f BasicScene::link_source_position(int index){
    Eigen::Vector3f rotation_v = cyls[index]->GetRotation() * link_len;
    return link_center_position(index) - rotation_v;
}

Eigen::Vector3f BasicScene::GetSpherePos()
{
      Eigen::Vector3f l = Eigen::Vector3f(1.6f,0,0);
      Eigen::Vector3f res;
      res = cyls[tipIndex]->GetRotation()*l;   
      return res;  
}






