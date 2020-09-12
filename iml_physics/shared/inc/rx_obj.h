/*!
  @file rx_obj.h
	
  @brief OBJ/MTL File Input/Output
 
  @author Makoto Fujisawa
  @date   2011
*/

#ifndef _RX_OBJ_H_
#define _RX_OBJ_H_


//-----------------------------------------------------------------------------
// Include Files
//-----------------------------------------------------------------------------
#include "rx_mesh.h"

//-----------------------------------------------------------------------------
// Name Space
//-----------------------------------------------------------------------------
using namespace std;


//-----------------------------------------------------------------------------
// rxOBJ�N���X�̐錾 - OBJ�`���̓ǂݍ���
//-----------------------------------------------------------------------------
class rxOBJ
{
	rxMTL m_mapMaterials;	//!< ���x���ƃf�[�^�̃}�b�v
	string m_strCurrentMat;	//!< ���݂̃f�[�^���������x��

	//vector<rxMaterialOBJ> m_vMaterials;
	//int m_iCurrentMat;

public:
	//! �R���X�g���N�^
	rxOBJ();
	//! �f�X�g���N�^
	~rxOBJ();

	/*!
	 * OBJ�t�@�C���ǂݍ���
	 * @param[in] file_name �t�@�C����(�t���p�X)
	 * @param[out] vrts ���_���W
	 * @param[out] vnms ���_�@��
	 * @param[out] poly �|���S��
	 * @param[out] mats �ގ����
	 * @param[in] triangle �|���S���̎O�p�`�����t���O
	 */
	bool Read(string file_name, vector<Vec3> &vrts, vector<Vec3> &vnms, vector<rxFace> &plys, rxMTL &mats, bool triangle = true);

	/*!
	 * OBJ�t�@�C����������
	 * @param[in] file_name �t�@�C����(�t���p�X)
	 * @param[in] vrts ���_���W
	 * @param[in] vnms ���_�@��
	 * @param[in] plys �|���S��
	 * @param[in] mats �ގ����
	 */
	bool Save(string file_name, const vector<Vec3> &vrts, const vector<Vec3> &vnms, const vector<rxFace> &plys, const rxMTL &mats);

	//! �ގ����X�g�̎擾
	rxMTL GetMaterials(void){ return m_mapMaterials; }

private:
	int loadFace(string &buf, vector<int> &vidxs, vector<int> &nidxs, vector<int> &tidxs);
	int loadMTL(const string &mtl_fn);
	int saveMTL(const string &mtl_fn, const rxMTL &mats);
};



#endif // _RX_OBJ_H_
