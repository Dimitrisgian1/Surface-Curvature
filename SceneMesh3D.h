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
#define FLAG_SHOW_TASK1		64

void ChooseRandTris(std::vector<vvr::Triangle> &triangles, std::vector<int> &rand_tri_indices);
void FixRandTri(std::vector<vvr::Triangle> &triangles, std::vector<int> &rand_tri_indices, std::vector<vec> &verts);
vec ChooseRandTriVert(vvr::Triangle &triangle, vec &p2, vec &p3);
void FindAdjacentTriangle(std::vector<vvr::Triangle> &triangles, vec p1, vec p2, vec p3, unsigned &tri_adj_index, vec &opp_ver);
void FixManyTris(std::vector<vvr::Triangle> &triangles, std::vector<int> &rand_tri_indices, std::vector<vec> &verts);
float angle(vec P1, vec P2, vec P3);
void StoreNeighbourVertices(std::vector<vvr::Triangle> &triangles, vec P, std::vector<math::vec> &NeigbourVerts);
vec FindVertNormal(std::vector<vvr::Triangle> &triangles, vec P);
void StoreNeighbourTris(std::vector<vvr::Triangle> &triangles, vec P, std::vector<vvr::Triangle> &NeigbourTris);
void Curvature(vec P, std::vector<vvr::Triangle> &NeigbourTris);


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
	void Task1();

private:
    int m_style_flag;
    float m_plane_d;
    vvr::Canvas2D m_canvas;
    vvr::Colour m_obj_col;
    vvr::Mesh m_model_original, m_model, test_model;
    vvr::Box3D m_aabb;
	math::vec m_center_mass;
	std::vector<math::vec> VertNormals;
};
