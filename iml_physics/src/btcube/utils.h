/*! @file utils.h
	
	@brief 様々な関数
 
	@author Makoto Fujisawa
	@date 2012
*/

#ifndef _RX_UTILS_H_
#define _RX_UTILS_H_

#pragma warning (disable: 4101)

//-----------------------------------------------------------------------------
// インクルードファイル
//-----------------------------------------------------------------------------
#include <fstream>
#include <iostream>

#include <string>
#include <GL/glew.h>
#include <GL/glut.h>

#include "rx_utility.h"		// Vec3や様々な関数
#include "rx_trackball.h"	// 回転，平行移動

#include "rx_shadow_gl.h"


#include <btBulletDynamicsCommon.h>  

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBody.h>


using namespace std;

//-----------------------------------------------------------------------------
// 定数・定義
//-----------------------------------------------------------------------------
#define RX_MAX_CONTACTS 1024

inline std::ostream &operator<<(std::ostream &out, const btVector3 &a)
{
	return out << "(" << a[0] << ", " << a[1] << ", " << a[2] << ")" ;
}


inline std::istream &operator>>(std::istream &in, btVector3& a)
{
	return in >> a[0] >> a[1] >> a[2] ;
}

//-----------------------------------------------------------------------------
// OpenGL
//-----------------------------------------------------------------------------
/*!
 * xyz軸描画(x軸:赤,y軸:緑,z軸:青)
 * @param[in] len 軸の長さ
 */
inline int DrawAxis(double len, double line_width = 5.0)
{
	glLineWidth(line_width);

	// x軸
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	glVertex3d(0.0, 0.0, 0.0);
	glVertex3d(len, 0.0, 0.0);
	glEnd();

	// y軸
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
	glVertex3d(0.0, 0.0, 0.0);
	glVertex3d(0.0, len, 0.0);
	glEnd();

	// z軸
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
	glVertex3d(0.0, 0.0, 0.0);
	glVertex3d(0.0, 0.0, len);
	glEnd();

	return 1;
}

/*!
 * 円筒描画
 * @param[in] rad,len 半径と中心軸方向長さ
 * @param[in] axis 軸方向
 * @param[in] slices  ポリゴン近似する際の分割数
 */
static void DrawCylinder(double rad, double len, int axis, int slices)
{
	GLUquadricObj *qobj;
	qobj = gluNewQuadric();

	glPushMatrix();
	switch(axis){
	case 0:
		glRotatef(-90.0, 0.0, 1.0, 0.0);
		glTranslatef(0.0, 0.0, -0.5*len);
		break;
	case 1:
		glRotatef(-90.0, 1.0, 0.0, 0.0);
		glTranslatef(0.0, 0.0, -0.5*len);
		break;
	case 2:
		glTranslatef(0.0, 0.0, -0.5*len);
		break;
	default:
		glTranslatef(0.0, 0.0, -0.5*len);
	}

	gluQuadricDrawStyle(qobj, GLU_FILL);
	gluQuadricNormals(qobj, GLU_SMOOTH);
	gluCylinder(qobj, rad, rad, len, slices, slices);

	glPushMatrix();
	glRotatef(180.0, 1.0, 0.0, 0.0);
	gluDisk(qobj, 0.0, rad, slices, slices);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(0.0, 0.0, len);
	gluDisk(qobj, 0.0, rad, slices, slices);
	glPopMatrix();

	glPopMatrix();
}

/*!
 * カプセル描画(円筒の両端に半球をつけた形)
 * @param[in] rad,len 半径と中心軸方向長さ
 * @param[in] axis 軸方向
 * @param[in] slices  ポリゴン近似する際の分割数
 */
static void DrawCapsule(double rad, double len, int axis, int slices)
{
	GLUquadricObj *qobj;
	qobj = gluNewQuadric();

	glPushMatrix();
	switch(axis){
	case 0:
		glRotatef(-90.0, 0.0, 1.0, 0.0);
		glTranslatef(0.0, 0.0, -0.5*len);
		break;
	case 1:
		glRotatef(-90.0, 1.0, 0.0, 0.0);
		glTranslatef(0.0, 0.0, -0.5*len);
		break;
	case 2:
		glTranslatef(0.0, 0.0, -0.5*len);
		break;
	default:
		glTranslatef(0.0, 0.0, -0.5*len);
	}

	gluQuadricDrawStyle(qobj, GLU_FILL);
	gluQuadricNormals(qobj, GLU_SMOOTH);
	gluCylinder(qobj, rad, rad, len, slices, slices);

	glPushMatrix();
	glutSolidSphere(rad, slices, slices);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(0.0, 0.0, len);
	glutSolidSphere(rad, slices, slices);
	glPopMatrix();

	glPopMatrix();

}



class TriangleDrawCallback : public btTriangleCallback
{
public:
	TriangleDrawCallback(){}
	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex)
	{
		btVector3 n = ((triangle[0]-triangle[1]).cross(triangle[0]-triangle[2])).normalize();
		glNormal3f(n[0], n[1], n[2]);

		glBegin(GL_TRIANGLES);
		glVertex3d(triangle[0][0], triangle[0][1], triangle[0][2]);
		glVertex3d(triangle[1][0], triangle[1][1], triangle[1][2]);
		glVertex3d(triangle[2][0], triangle[2][1], triangle[2][2]);
		glEnd();
	}
};


static inline void DrawBulletSoftBody(btSoftBody* sbody)
{
	if(sbody->m_faces.size() == 0){
		glBegin(GL_LINE_STRIP);
		for(int i = 0; i < sbody->m_nodes.size(); ++i){
			const btSoftBody::Node &node = sbody->m_nodes[i];
			btVector3 p = node.m_x;
			glVertex3f(p.x(), p.y(), p.z());
		}
		glEnd();
	}

	for(int i = 0; i < sbody->m_faces.size(); ++i){
		const btSoftBody::Face &face = sbody->m_faces[i];

		glBegin(GL_POLYGON);
		for(int j = 0; j < 3; ++j){
			const btSoftBody::Node* node = face.m_n[j];
			btVector3 p = node->m_x;
			btVector3 n = node->m_n;
			glNormal3f(n.x(), n.y(), n.z());
			glVertex3f(p.x(), p.y(), p.z());

		}
		glEnd();
	}
}

/*!
* Bulletの衝突形状を描画
* @param[in] shape 衝突形状
*/
static void DrawBulletShape(const btCollisionShape *shape, btVector3 &world_min, btVector3 &world_max)
{
	int shapetype = shape->getShapeType();

	glPushMatrix();

	// 形状の種類ごとに描画
	if(shapetype == BOX_SHAPE_PROXYTYPE){
		// ボックス形状
		const btBoxShape* box = static_cast<const btBoxShape*>(shape);
		btVector3 half_extent = box->getHalfExtentsWithMargin();
		glScaled(2*half_extent[0], 2*half_extent[1], 2*half_extent[2]);
		glutSolidCube(1.0);
	} else if(shapetype == SPHERE_SHAPE_PROXYTYPE){
		// 球形状
		const btSphereShape* sphere = static_cast<const btSphereShape*>(shape);
		double rad = sphere->getRadius();
		glutSolidSphere(rad, 32, 32);
	} else if(shapetype == CYLINDER_SHAPE_PROXYTYPE){
		// 円筒形状
		const btCylinderShape* cylinder = static_cast<const btCylinderShape*>(shape);
		double rad = cylinder->getRadius();
		int up_axis = cylinder->getUpAxis();
		double len = cylinder->getHalfExtentsWithMargin()[up_axis]*2;
		DrawCylinder(rad, len, up_axis, 8);
	} else if(shapetype == CAPSULE_SHAPE_PROXYTYPE){
		// カプセル形状
		const btCapsuleShape* capsule = static_cast<const btCapsuleShape*>(shape);
		double rad = capsule->getRadius();
		int up_axis = capsule->getUpAxis();
		double len = capsule->getHalfHeight()*2;
		DrawCapsule(rad, len, up_axis, 16);
	} else if(shapetype == TRIANGLE_MESH_SHAPE_PROXYTYPE){
		// 三角形メッシュ
		const btBvhTriangleMeshShape* mesh = static_cast<const btBvhTriangleMeshShape*>(shape);
		TriangleDrawCallback draw_callback;
		mesh->processAllTriangles(&draw_callback, world_min, world_max);
	} else if(shapetype == GIMPACT_SHAPE_PROXYTYPE){
		// 三角形メッシュ(GIMPACT)
		const btGImpactMeshShape* mesh = static_cast<const btGImpactMeshShape*>(shape);
		TriangleDrawCallback draw_callback;
		mesh->processAllTriangles(&draw_callback, world_min, world_max);
	} else if(shapetype == COMPOUND_SHAPE_PROXYTYPE){
		// 複合形状
		const btCompoundShape* compound = static_cast<const btCompoundShape*>(shape);
		int num_child = compound->getNumChildShapes();
		btScalar mc[16];
		for(int j = 0; j < num_child; ++j){
			// Compound Shapeを構成するchild shapeを再帰的にDrawBulletShapeに渡す
			compound->getChildTransform(j).getOpenGLMatrix(mc);
			glPushMatrix();
			glMultMatrixf(mc);
			DrawBulletShape(compound->getChildShape(j), world_min, world_max);
			glPopMatrix();
		}
	} else{
		glutSolidSphere(1.0, 10, 10);
	}

	glPopMatrix();
}


#endif // #ifndef _RX_UTILS_H_