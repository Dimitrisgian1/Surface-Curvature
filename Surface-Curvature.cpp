#include "SceneMesh3D.h"
#include <stdlib.h>

// My global mesh
//Mesh my_mesh;

Mesh3DScene::Mesh3DScene()
{
    //! Load settings.
    vvr::Shape::DEF_LINE_WIDTH = 4;
    vvr::Shape::DEF_POINT_SIZE = 10;
    m_perspective_proj = true;
    m_bg_col = vvr::Colour("768E77");
    m_obj_col = vvr::Colour("454545");
    const std::string objDir = vvr::getBasePath() + "resources/obj/";
    const std::string objFile = objDir + "suzanne.obj";
    m_model_original = vvr::Mesh(objFile);
    reset();
}

void Mesh3DScene::reset()
{
    Scene::reset();

    //! Define what will be vissible by default
    m_style_flag = 0;
    m_style_flag |= FLAG_SHOW_SOLID;
    m_style_flag |= FLAG_SHOW_WIRE;
    m_style_flag |= FLAG_SHOW_AXES;
    m_style_flag |= FLAG_SHOW_AABB;
    //m_style_flag |= FLAG_SHOW_PLANE;
}

void Mesh3DScene::resize()
{
    //! By Making `first_pass` static and initializing it to true,
    //! we make sure that the if block will be executed only once.

    static bool first_pass = true;

    if (first_pass)
    {
        m_model_original.setBigSize(getSceneWidth() / 2);
        m_model_original.update();
        m_model = m_model_original;
		Tasks();
        first_pass = false;
    }
}

void Mesh3DScene::Tasks()
{
	FindCenterMass(m_model.getVertices(), m_center_mass);

	FindAABB(m_model.getVertices(), m_aabb);

	std::vector<int> rand_tri_indices;
	ChooseRandTri(m_model.getTriangles(), rand_tri_indices);
}

void Mesh3DScene::keyEvent(unsigned char key, bool up, int modif)
{
    Scene::keyEvent(key, up, modif);
    key = tolower(key);

    switch (key)
    {
    case 's': m_style_flag ^= FLAG_SHOW_SOLID; break;
    case 'w': m_style_flag ^= FLAG_SHOW_WIRE; break;
    case 'n': m_style_flag ^= FLAG_SHOW_NORMALS; break;
    case 'a': m_style_flag ^= FLAG_SHOW_AXES; break;
    case 'p': m_style_flag ^= FLAG_SHOW_PLANE; break;
    case 'b': m_style_flag ^= FLAG_SHOW_AABB; break;
    }
}


void Mesh3DScene::draw()
{

	if (m_style_flag & FLAG_SHOW_SOLID) m_model.draw(m_obj_col, vvr::SOLID);
	if (m_style_flag & FLAG_SHOW_WIRE) m_model.draw(vvr::Colour::black, vvr::WIRE);
	if (m_style_flag & FLAG_SHOW_NORMALS) m_model.draw(vvr::Colour::black, vvr::NORMALS);
	if (m_style_flag & FLAG_SHOW_AXES) m_model.draw(vvr::Colour::black, vvr::AXES);

	// Draw my new green split mesh, solid and wireframe
	//my_mesh.draw(Colour::green, SOLID);
	//my_mesh.draw(Colour::green, WIRE);

	//! Draw center mass
	vvr::Point3D(m_center_mass.x, m_center_mass.y, m_center_mass.z, vvr::Colour::red).draw();

	//! Draw AABB
	if (m_style_flag & FLAG_SHOW_AABB) {
		m_aabb.setColour(vvr::Colour::black);
		m_aabb.setTransparency(1);
		m_aabb.draw();
	}
	
}

int main(int argc, char* argv[])
{
    try {
        return vvr::mainLoop(argc, argv, new Mesh3DScene);
    }
    catch (std::string exc) {
		std::cerr << exc << std::endl;
        return 1;
    }
    catch (...)
    {
		std::cerr << "Unknown exception" << std::endl;
        return 1;
    }
}

void FindCenterMass(std::vector<vec> &vertices, vec &cm)
{
	cm.x = 0;
	cm.y = 0;
	cm.z = 0;
	for (int i = 0; i < vertices.size(); i++)
	{
		cm.x += vertices[i].x;
		cm.y += vertices[i].y;
		cm.z += vertices[i].z;
	}

	cm.x /= vertices.size();
	cm.y /= vertices.size();
	cm.z /= vertices.size();
}

void FindAABB(std::vector<vec> &vertices, vvr::Box3D &aabb)
{
	vvr::Point3D V_min(1000, 1000, 1000), V_max(0, 0, 0);

	for (int i = 0; i < vertices.size(); i++)
	{
		V_min.x = (vertices[i].x < V_min.x) ? vertices[i].x : V_min.x;
		V_min.y = (vertices[i].y < V_min.y) ? vertices[i].y : V_min.y;
		V_min.z = (vertices[i].z < V_min.z) ? vertices[i].z : V_min.z;

		V_max.x = (vertices[i].x > V_max.x) ? vertices[i].x : V_max.x;
		V_max.y = (vertices[i].y > V_max.y) ? vertices[i].y : V_max.y;
		V_max.z = (vertices[i].z > V_max.z) ? vertices[i].z : V_max.z;
	}

	aabb.x1 = V_min.x;
	aabb.y1 = V_min.y;
	aabb.z1 = V_min.z;
	aabb.x2 = V_max.x;
	aabb.y2 = V_max.y;
	aabb.z2 = V_max.z;
}

void ChooseRandTri(std::vector<vvr::Triangle> &triangles, std::vector<int> &rand_tri_indices)
{
	for (int i = 0; i < triangles.size() / 10; i++)
	{
		srand(i);
		int rand_index = rand() % triangles.size();
		rand_tri_indices.push_back(rand_index);
		//triangles.erase(triangles.begin() + rand_tri_indices[i]);
	}
}

//void FixRandTri(std::vector<int> &rand_tri_indices)
//{
//
