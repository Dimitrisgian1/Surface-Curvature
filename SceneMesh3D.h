#include <VVRScene/canvas.h>
#include <VVRScene/mesh.h>
#include <VVRScene/settings.h>
#include <VVRScene/utils.h>
#include <MathGeoLib.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <set>

#define FLAG_SHOW_AXES       1
#define FLAG_SHOW_WIRE       2
#define FLAG_SHOW_SOLID      4
#define FLAG_SHOW_NORMALS    8
#define FLAG_SHOW_PLANE     16
#define FLAG_SHOW_AABB      32

void FindCenterMass(std::vector<vec> &vertices, vec &cm);
void FindAABB(std::vector<vec> &vertices, vvr::Box3D &aabb);
void ChooseRandTri(std::vector<vvr::Triangle> &triangles, std::vector<int> &rand_tri_indices);

class Mesh3DScene : public vvr::Scene
{
public:
    Mesh3DScene();
    const char* getName() const { return "3D Scene"; }
    void keyEvent(unsigned char key, bool up, int modif) override;

private:
    void draw() override;
    void reset() override;
    void resize() override;
	void Tasks();

private:
    int m_style_flag;
    float m_plane_d;
    vvr::Canvas2D m_canvas;
    vvr::Colour m_obj_col;
    vvr::Mesh m_model_original, m_model;
    vvr::Box3D m_aabb;
	math::vec m_center_mass;
};
