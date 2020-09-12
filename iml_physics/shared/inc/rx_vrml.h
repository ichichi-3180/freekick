/*!
  @file rx_vrml.h

  @brief VRML Input/Output
	- Shape��IndexedFaceSet�̂ݑΉ�

  @author Makoto Fujisawa
  @date   2011
*/

#ifndef _RX_VRML_H_
#define _RX_VRML_H_


//-----------------------------------------------------------------------------
// Include Files
//-----------------------------------------------------------------------------
#include "rx_mesh.h"

//-----------------------------------------------------------------------------
// Name Space
//-----------------------------------------------------------------------------
using namespace std;


//-----------------------------------------------------------------------------
// VRML�N���X�̐錾 - VRML�`���̓ǂݍ���
//-----------------------------------------------------------------------------
class rxVRML
{
public:
	//! �R���X�g���N�^
	rxVRML();
	//! �f�X�g���N�^
	~rxVRML();

	/*!
	 * VRML�t�@�C���ǂݍ���
	 * @param[in] file_name �t�@�C����(�t���p�X)
	 * @param[out] vrts ���_���W
	 * @param[out] vnms ���_�@��
	 * @param[out] poly �|���S��
	 * @param[out] mats �ގ����
	 * @param[in] triangle �|���S���̎O�p�`�����t���O
	 */
	bool Read(string file_name, vector<Vec3> &vrts, vector<Vec3> &vnms, vector<rxFace> &plys, rxMTL &mats, bool tri = true);

	/*!
	 * VRML�t�@�C����������
	 * @param[in] file_name �t�@�C����(�t���p�X)
	 * @param[in] vrts ���_���W
	 * @param[in] vnms ���_�@��
	 * @param[in] plys �|���S��
	 * @param[in] mats �ގ����
	 */
	bool Save(string file_name, const vector<Vec3> &vrts, const vector<Vec3> &vnms, const vector<rxFace> &plys, const rxMTL &mats);

};




#endif // _RX_VRML_H_
