/*!
  @file rx_model.h
	
  @brief 3D���f���t�@�C�� Input/Output
 
  @author Makoto Fujisawa
  @date   2011
*/


#ifndef _RX_MODEL_H_
#define _RX_MODEL_H_


//-----------------------------------------------------------------------------
// Include Files
//-----------------------------------------------------------------------------
#include "rx_mesh.h"

#include "rx_obj.h"
#include "rx_dxf.h"
#include "rx_vrml.h"
#include "rx_3ds.h"
#include "rx_stl.h"
#include "rx_ply.h"
#include "rx_off.h"



//-----------------------------------------------------------------------------
// 3D���f���t�@�C������/�o�͊֐�
//-----------------------------------------------------------------------------
namespace RxModel
{
void Read(const string &filename, rxPolygons &polys);
void Save(const string &filename, rxPolygons &polys);

void ReadOBJ(const string filename, rxPolygons &polys, Vec3 cen, Vec3 ext, Vec3 ang);
void SaveOBJ(const string filename, rxPolygons &polys);

void ReadDXF(const string filename, rxPolygons &polys, Vec3 cen, Vec3 ext, Vec3 ang);
void SaveDXF(const string filename, rxPolygons &polys);

void ReadVRML(const string filename, rxPolygons &polys, Vec3 cen, Vec3 ext, Vec3 ang);
void SaveVRML(const string filename, rxPolygons &polys);

void Read3DS(const string filename, rxPolygons &polys, Vec3 cen, Vec3 ext, Vec3 ang);
void Save3DS(const string filename, rxPolygons &polys);

void ReadSTL(const string filename, rxPolygons &polys, Vec3 cen, Vec3 ext, Vec3 ang);
void SaveSTL(const string filename, rxPolygons &polys);

void ReadPLY(const string filename, rxPolygons &polys, Vec3 cen, Vec3 ext, Vec3 ang);
void SavePLY(const string filename, rxPolygons &polys);

void ReadOFF(const string filename, rxPolygons &polys, Vec3 cen, Vec3 ext, Vec3 ang);
//void SaveOFF(const string filename, rxPolygons &polys);
};


#endif // _RX_MODEL_H_
