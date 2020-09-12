/*!
  @file rx_dxf.cpp

  @brief DXF File Input
		- 3DFACE�̂�
 
  @author Makoto Fujisawa
  @date   2011
*/

#ifndef _RX_DXF_H_
#define _RX_DXF_H_


//-----------------------------------------------------------------------------
// Include Files
//-----------------------------------------------------------------------------
#include "rx_mesh.h"

//-----------------------------------------------------------------------------
// Name Space
//-----------------------------------------------------------------------------
using namespace std;


//-----------------------------------------------------------------------------
// rxDXF�N���X�̐錾 - DXF�`���̓ǂݍ���
//-----------------------------------------------------------------------------
class rxDXF
{
public:
	//! �R���X�g���N�^
	rxDXF();
	//! �f�X�g���N�^
	~rxDXF();

	/*!
	 * DXF�t�@�C���ǂݍ���
	 * @param[in] file_name �t�@�C����(�t���p�X)
	 * @param[out] vrts ���_���W
	 * @param[out] vnms ���_�@��
	 * @param[out] poly �|���S��
	 * @param[out] mats �ގ����
	 * @param[in] triangle �|���S���̎O�p�`�����t���O
	 */
	bool Read(string file_name, vector<Vec3> &vrts, vector<Vec3> &vnms, vector<rxFace> &plys, bool triangle = true);

	/*!
	 * DXF�t�@�C����������
	 * @param[in] file_name �t�@�C����(�t���p�X)
	 * @param[in] vrts ���_���W
	 * @param[in] plys �|���S��
	 */
	bool Save(string file_name, const vector<Vec3> &vrts, const vector<rxFace> &plys);

private:
	// �Z�N�V����
	enum{
		RX_DXF_SECTION_HEADER = 1, 
		RX_DXF_SECTION_CLASSES, 
		RX_DXF_SECTION_TABLES, 
		RX_DXF_SECTION_BLOCKS, 
		RX_DXF_SECTION_ENTITES, 
		RX_DXF_SECTION_OBJECTS, 
		RX_DXF_SECTION_THUMBNAILIMAGE, 
	};

	// �G���e�B�e�B
	enum{
		RX_DXF_ENTITY_3DFACE = 1, 
		RX_DXF_ENTITY_3DSOLID, 
		RX_DXF_ENTITY_CIRCLE, 
		RX_DXF_ENTITY_ELLIPSE, 
		RX_DXF_ENTITY_LINE, 
		RX_DXF_ENTITY_MESH, 
		RX_DXF_ENTITY_POINT, 
		RX_DXF_ENTITY_POLYLINE, 
		RX_DXF_ENTITY_SPLINE, 
		RX_DXF_ENTITY_TEXT, 
		RX_DXF_ENTITY_VERTEX, 
	};

	map<string, int> m_mapSection;
	map<string, int> m_mapEntity;
};




#endif // _RX_VRML_H_
