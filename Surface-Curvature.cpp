#include "SceneMesh3D.h"
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

//My global mesh
//vvr::Mesh my_mesh;

Mesh3DScene::Mesh3DScene()
{
    //! Load settings.
    vvr::Shape::DEF_LINE_WIDTH = 4;
    vvr::Shape::DEF_POINT_SIZE = 10;
    m_perspective_proj = true;
    m_bg_col = vvr::Colour("768E77");
    m_obj_col = vvr::Colour("454545");
    const std::string objDir = vvr::getBasePath() + "resources/obj/";
    const std::string objFile = objDir + "dragon_low_low.obj";
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
	//m_style_flag |= FLAG_SHOW_TASK1; 
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
	case '1': m_style_flag ^= FLAG_SHOW_TASK1; Task1(); break;
    }
}

void Mesh3DScene::Tasks()
{
	if (m_style_flag & FLAG_SHOW_TASK1) Task1();
}

void Mesh3DScene::draw()
{
	//vvr::Triangle3D t3d(
	//	tri.v1.x, tri.v1.y, tri.v1.z,
	//	triv2.x, tri.v2.y, tri.v2.z,
	//	tri.v3.x, tri.v3.y, tri.v3.z,
	//	Colour::green);
	//t3d.draw();
	//vvr::Point3D(opp_ver.x, opp_ver.y, opp_ver.z, vvr::Colour::red).draw();

	if (m_style_flag & FLAG_SHOW_SOLID) m_model.draw(m_obj_col, vvr::SOLID);
	if (m_style_flag & FLAG_SHOW_WIRE) m_model.draw(vvr::Colour::black, vvr::WIRE);
	if (m_style_flag & FLAG_SHOW_NORMALS) m_model.draw(vvr::Colour::black, vvr::NORMALS);
	if (m_style_flag & FLAG_SHOW_AXES) m_model.draw(vvr::Colour::black, vvr::AXES);

	//! Draw center mass
	//vvr::Point3D(m_center_mass.x, m_center_mass.y, m_center_mass.z, vvr::Colour::red).draw();

	//! Draw AABB
	if (m_style_flag & FLAG_SHOW_AABB) {
		m_aabb.setColour(vvr::Colour::black);
		m_aabb.setTransparency(1);
		m_aabb.draw();
	}
	//for (int i = 0; i < TriNormals.size(); i++)
	//	vvr::Point3D(TriNormals[i].x*100, TriNormals[i].y * 100, TriNormals[i].z * 100, vvr::Colour::black).draw();
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

void Mesh3DScene::Task1()
{
	std::vector<int> rand_tri_indices;
	std::vector<vvr::Triangle> &triangles = m_model.getTriangles();
	std::vector<vec>		   &verts = m_model.getVertices();

	std::vector<vvr::Triangle> NeighbourTris;

	StoreNeighbourTris(triangles, verts[0], NeighbourTris);
	Curvature(verts[0], NeighbourTris);
	FixManyTris(triangles, rand_tri_indices, verts);
	m_model.update();
}

void FixManyTris(std::vector<vvr::Triangle> &triangles, std::vector<int> &rand_tri_indices, std::vector<vec> &verts)
{
	ChooseRandTris(triangles, rand_tri_indices);
	FixRandTri(triangles, rand_tri_indices, verts);
}

void ChooseRandTris(std::vector<vvr::Triangle> &triangles, std::vector<int> &rand_tri_indices)
{
	for (int i = 0; i < 100; i++)///////////////////////////o ari8mos twn epanalhpsewn ka8orizei ton ari8mo twn tuxaiwn trigwnwn pou epilegw
	{
		//srand(i);
		int rand_index = rand() % triangles.size() - 1;
		rand_tri_indices.push_back(rand_index);
	}
}

void FixRandTri(std::vector<vvr::Triangle> &triangles, std::vector<int> &rand_tri_indices, std::vector<vec> &verts)
{
	int counter = 0;
	for (int i : rand_tri_indices)
	{
		unsigned tri_adj_index;
		vec P1, P2, P3, opp_ver;
		bool flag = 0;

		if (i > triangles.size()) i = rand() % triangles.size() - 1;

		P1 = ChooseRandTriVert(triangles[i], P2, P3);
		FindAdjacentTriangle(triangles, P1, P2, P3, tri_adj_index, opp_ver);

		//float AnglesBef[6] = { angle(P1, P2, P3),	   angle(P2, P1, P3),	   angle(P3, P1, P2), 
		//					   angle(opp_ver, P2, P3), angle(P2, P3, opp_ver), angle(P3, P2, opp_ver) };

		//float AnglesAft[6] = { angle(P1, P2, opp_ver), angle(opp_ver, P2, P1), angle(P2, P1, opp_ver),
		//					   angle(P1, opp_ver, P3), angle(opp_ver, P1, P3), angle(P3, P1, opp_ver) };

		//int counter;

		//for (int k = 0; k < 6; k++)
		//{
		//	counter = 0;
		//	for (int j = 0; j < 6; j++)
		//		if (AnglesBef[k] < AnglesAft[j]) counter++;
		//	if (counter == 6) flag = 1;
		//}

		if (angle(P1, P2, P3) + angle(opp_ver, P2, P3) > 200 * pi / 180) flag = 1;
		if (flag)
		{
			counter++;
			std::vector<vec> verts1 = { P1, P2, opp_ver };
			std::vector<vec> verts2 = { P1, P3, opp_ver };

			for (int l = 0; l < 3; l++)
				verts.push_back((verts1[l]));

			unsigned vi = verts.size() - 3;

			triangles.push_back(vvr::Triangle(&verts, vi, vi + 1, vi + 2));

			for (int l = 0; l < 3; l++)
				verts.push_back((verts2[l]));

			vi = verts.size() - 3;

			triangles.push_back(vvr::Triangle(&verts, vi, vi + 1, vi + 2));

			if (i > tri_adj_index)
			{
				triangles.erase(triangles.begin() + i);
				triangles.erase(triangles.begin() + tri_adj_index);
			}
			else
			{
				triangles.erase(triangles.begin() + tri_adj_index);
				triangles.erase(triangles.begin() + i);
			}
		}
	}
	std::cout << counter << std::endl;
}

vec ChooseRandTriVert(vvr::Triangle &triangle, vec &p2, vec &p3)
{
	std::vector<vec> vert;
	vert.push_back((triangle).v1());
	vert.push_back((triangle).v2());
	vert.push_back((triangle).v3());

	//srand(time(NULL));
	int rand_vert = rand() % 3;

	if (rand_vert == 0)
	{
		vec v2 = vert[rand_vert + 1];
		vec v3 = vert[rand_vert + 2];

		p2.Set(v2.x, v2.y, v2.z);
		p3.Set(v3.x, v3.y, v3.z);
	}

	if (rand_vert == 1)
	{
		vec v2 = vert[rand_vert - 1];
		vec v3 = vert[rand_vert + 1];

		p2.Set(v2.x, v2.y, v2.z);
		p3.Set(v3.x, v3.y, v3.z);
	}

	if (rand_vert == 2)
	{
		vec v2 = vert[rand_vert - 2];
		vec v3 = vert[rand_vert - 1];

		p2.Set(v2.x, v2.y, v2.z);
		p3.Set(v3.x, v3.y, v3.z);
	}
	return vert[rand_vert];
}

void FindAdjacentTriangle(std::vector<vvr::Triangle> &triangles, vec p1, vec p2, vec p3, unsigned &tri_adj_index, vec &opp_ver)
{
	for (int i = 0; i < triangles.size(); i++)		//oles oi periptwseis korufwn i = shmeiwn j
	{
		vec v1 = triangles[i].v1();
		vec v2 = triangles[i].v2();
		vec v3 = triangles[i].v3();

		if (p1.Equals(v1) || p1.Equals(v2) || p1.Equals(v3)) continue;

		if (p2.Equals(v1))		//v1 -> p2
		{
			if (p3.Equals(v2))	//v2 -> p3
			{
				opp_ver.Set(v3.x, v3.y, v3.z);
				tri_adj_index = i;
				return;
			}

			if (p3.Equals(v3))	//v3 -> p3
			{
				opp_ver.Set(v2.x, v2.y, v2.z);
				tri_adj_index = i;
				return;
			}
		}

		if (p2.Equals(v2))		//v2 -> p2
		{
			if (p3.Equals(v1))	//v1 -> p3
			{
				opp_ver.Set(v3.x, v3.y, v3.z);
				tri_adj_index = i;
				return;
			}

			if (p3.Equals(v3))	//v3 -> p3
			{
				opp_ver.Set(v1.x, v1.y, v1.z);
				tri_adj_index = i;
				return;
			}
		}

		if (p2.Equals(v3))		//v3 -> p2
		{
			if (p3.Equals(v1))	//v1 -> p3
			{
				opp_ver.Set(v2.x, v2.y, v2.z);
				tri_adj_index = i;
				return;
			}

			if (p3.Equals(v2))	//v2 -> p3
			{
				opp_ver.Set(v1.x, v1.y, v1.z);
				tri_adj_index = i;
				return;
			}
		}
	}
	return;
}

//returns the inner angle P2P1P3
float angle(vec P1, vec P2, vec P3)
{
	float dx21 = P2.x - P1.x;
	float dx31 = P3.x - P1.x;
	float dy21 = P2.y - P1.y;
	float dy31 = P3.y - P1.y;
	float m12 = sqrt(dx21*dx21 + dy21 * dy21);
	float m13 = sqrt(dx31*dx31 + dy31 * dy31);
	float theta = acos((dx21*dx31 + dy21 * dy31) / (m12 * m13));
	return theta;
}

//Finds all neigbouring vertices to one vertex and stores theim in NeigbourVerts 
void StoreNeighbourVertices(std::vector<vvr::Triangle> &triangles, vec P, std::vector<math::vec> &NeigbourVerts)
{
	for (int i = 0; i < triangles.size(); i++)
	{
		vec v1 = triangles[i].v1();
		vec v2 = triangles[i].v2();
		vec v3 = triangles[i].v3();

		if (P.Equals(v1))
		{
			NeigbourVerts.push_back(v2);
			NeigbourVerts.push_back(v3);
		}
		if (P.Equals(v2))
		{
			NeigbourVerts.push_back(v1);
			NeigbourVerts.push_back(v3);
		}
		if (P.Equals(v3))
		{
			NeigbourVerts.push_back(v1);
			NeigbourVerts.push_back(v2);
		}
	}
}

//Returns the normal of a vertex
vec FindVertNormal(std::vector<vvr::Triangle> &triangles, vec P)
{
	std::vector<vvr::Triangle> NeighbourTris;
	StoreNeighbourTris(triangles, P, NeighbourTris);

	vec normal(0, 0, 0);
	for (int i = 0; i < NeighbourTris.size(); i++)
	{
		vec P1 = triangles[i].v1();
		vec P2 = triangles[i].v2();
		vec P3 = triangles[i].v3();
		math::Triangle T(P1, P2, P3);
		normal += T.Area() * T.NormalCCW();//prose3e to auto sto laplacian mesh smoothing
	}
	normal.x = normal.x / normal.Length();
	normal.y = normal.y / normal.Length();
	normal.z = normal.z / normal.Length();

	return normal;
}

//this is how StoreNeighbourVertices is used, You cannot have the indices to the neighbours this way
void FindNeighbourVertices(std::vector<vvr::Triangle> &triangles, vec P)
{
	std::vector<math::vec> NeighbourVerts;
	StoreNeighbourVertices(triangles, P, NeighbourVerts);
}

//Finds all neigbouring triangles to one vertex and stores them in NeigbourTris 
void StoreNeighbourTris(std::vector<vvr::Triangle> &triangles, vec P, std::vector<vvr::Triangle> &NeigbourTris)
{
	for (int i = 0; i < triangles.size(); i++)
	{
		vec v1 = triangles[i].v1();
		vec v2 = triangles[i].v2();
		vec v3 = triangles[i].v3();

		if (P.Equals(v1) || P.Equals(v2) || P.Equals(v3))
			NeigbourTris.push_back(triangles[i]);
	}
}

void Curvature(vec P, std::vector<vvr::Triangle> &NeigbourTris)
{
	float Area = 0;
	vec MeanCurv(0,0,0);
	float GaussCurv = 2*pi;

	std::vector<vvr::Triangle> Pairs;
	std::vector<int> TriVertIsP;
	for (int i = 0; i < NeigbourTris.size(); i++)
	{
		vec v1 = NeigbourTris[i].v1();
		vec v2 = NeigbourTris[i].v2();
		vec v3 = NeigbourTris[i].v3();
		if (P.Equals(v1)) TriVertIsP.push_back(0);
		if (P.Equals(v2)) TriVertIsP.push_back(1);
		if (P.Equals(v3)) TriVertIsP.push_back(2);
	}

	for (int i = 0; i < NeigbourTris.size(); i++)
	{
		{
			for (int j = 0; j < NeigbourTris.size(); j++)
			{
				if (j == i) continue;
				std::vector<vec> vi = { NeigbourTris[i].v1(), NeigbourTris[i].v2(), NeigbourTris[i].v3() };
				std::vector<vec> vj = { NeigbourTris[j].v1(), NeigbourTris[j].v2(), NeigbourTris[j].v3() };

				vec vk = vi[(TriVertIsP[i] + 1) % 3];	 //second common point of triangle i

				if (vk.Equals(vj[(TriVertIsP[j] + 1) % 3]))//second common point of triangle j
				{
					vec vA = vi[(TriVertIsP[i] + 2) % 3];//third (uncommon) point of triangle i
					vec vB = vi[TriVertIsP[i]];			 //first common point of triangle i
					vec vC = vj[(TriVertIsP[j] + 2) % 3];//third (uncommon) point of triangle j

					float a = angle(vA, vB, vk);
					float b = angle(vC, vB, vk);
					Area += (1 / tan(a)) + (1 / tan(b));
					Area *= (vB.DistanceSq(vk)) / 8;
					
					MeanCurv.x += (1 / tan(a)) + (1 / tan(b)) * (vk.x - P.x);
					MeanCurv.y += (1 / tan(a)) + (1 / tan(b)) * (vk.y - P.y);
					MeanCurv.z += (1 / tan(a)) + (1 / tan(b)) * (vk.z - P.z);

					GaussCurv -= angle(vB, vk, vA);
				}

				if (vk.Equals(vj[(TriVertIsP[j] + 2) % 3]))//second common point of triangle j
				{
					vec vA = vi[(TriVertIsP[i] + 2) % 3];//third (uncommon) point of triangle i
					vec vB = vi[TriVertIsP[i]];			 //first common point of triangle i
					vec vC = vj[(TriVertIsP[j] + 1) % 3];//third (uncommon) point of triangle j

					float a = angle(vA, vB, vk);
					float b = angle(vC, vB, vk);
					Area += (1 / tan(a)) + (1 / tan(b));
					Area *= (vB.DistanceSq(vk)) / 8;

					MeanCurv.x += (1 / tan(a)) + (1 / tan(b)) * (vk.x - P.x);
					MeanCurv.y += (1 / tan(a)) + (1 / tan(b)) * (vk.y - P.y);
					MeanCurv.z += (1 / tan(a)) + (1 / tan(b)) * (vk.z - P.z);

					GaussCurv -= angle(vB, vk, vA);
				}

				vk = vi[(TriVertIsP[i] + 2) % 3];	 //second common point of triangle i

				if (vk.Equals(vj[(TriVertIsP[j] + 1) % 3]))//second common point of triangle j
				{
					vec vA = vi[(TriVertIsP[i] + 1) % 3];//third (uncommon) point of triangle i
					vec vB = vi[TriVertIsP[i]];			 //first common point of triangle i
					vec vC = vj[(TriVertIsP[j] + 2) % 3];//third (uncommon) point of triangle j

					float a = angle(vA, vB, vk);
					float b = angle(vC, vB, vk);
					Area += (1 / tan(a)) + (1 / tan(b));
					Area *= (vB.DistanceSq(vk)) / 8;

					MeanCurv.x += (1 / tan(a)) + (1 / tan(b)) * (vk.x - P.x);
					MeanCurv.y += (1 / tan(a)) + (1 / tan(b)) * (vk.y - P.y);
					MeanCurv.z += (1 / tan(a)) + (1 / tan(b)) * (vk.z - P.z);

					GaussCurv -= angle(vB, vk, vA);
				}

				if (vk.Equals(vj[(TriVertIsP[j] + 2) % 3]))//second common point of triangle j
				{
					vec vA = vi[(TriVertIsP[i] + 2) % 3];//third (uncommon) point of triangle i
					vec vB = vi[TriVertIsP[i]];			 //first common point of triangle i
					vec vC = vj[(TriVertIsP[j] + 1) % 3];//third (uncommon) point of triangle j

					float a = angle(vA, vB, vk);
					float b = angle(vC, vB, vk);
					Area += (1 / tan(a)) + (1 / tan(b));
					Area *= (vB.DistanceSq(vk)) / 8;

					MeanCurv.x += (1 / tan(a)) + (1 / tan(b)) * (vk.x - P.x);
					MeanCurv.y += (1 / tan(a)) + (1 / tan(b)) * (vk.y - P.y);
					MeanCurv.z += (1 / tan(a)) + (1 / tan(b)) * (vk.z - P.z);

					GaussCurv -= angle(vB, vk, vA);
				}
			}
		}
	}
	MeanCurv /= (4 * Area);
	GaussCurv /= Area;
}
