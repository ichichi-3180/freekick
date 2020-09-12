/*!
  @file rx_stl.h
	
  @brief STL File Input/Output
 
  @author Makoto Fujisawa
  @date   2011
*/

#ifndef _RX_STL_H_
#define _RX_STL_H_


//-----------------------------------------------------------------------------
// Include Files
//-----------------------------------------------------------------------------
#include "rx_mesh.h"

//-----------------------------------------------------------------------------
// Name Space
//-----------------------------------------------------------------------------
using namespace std;


//-----------------------------------------------------------------------------
// rxSTL�N���X�̐錾 - STL�`���̓ǂݍ���
//-----------------------------------------------------------------------------
class rxSTL
{
public:
	//! �R���X�g���N�^
	rxSTL();
	//! �f�X�g���N�^
	~rxSTL();

	/*!
	 * STL�t�@�C���ǂݍ���
	 * @param[in] file_name �t�@�C����(�t���p�X)
	 * @param[out] vrts ���_���W
	 * @param[out] vnms ���_�@��
	 * @param[out] poly �|���S��
	 * @param[in] vertex_integration ���_�����t���O
	 * @param[in] vertex_normal ���_�@���v�Z�t���O
	 */
	bool Read(string file_name, vector<Vec3> &vrts, vector<Vec3> &vnms, vector<rxFace> &plys, 
			  bool vertex_integration = true, bool vertex_normal = true);

	/*!
	 * STL�t�@�C����������(������)
	 * @param[in] file_name �t�@�C����(�t���p�X)
	 * @param[in] vrts ���_���W
	 * @param[in] plys �|���S��
	 * @param[in] binary �o�C�i���t�H�[�}�b�g�ł̕ۑ��t���O
	 */
	bool Save(string file_name, const vector<Vec3> &vrts, const vector<rxFace> &plys, 
			  bool binary = false);

protected:
	//! ASCII�t�H�[�}�b�g��STL�t�@�C���ǂݍ���
	bool readAsciiData(ifstream &file, vector<Vec3> &vrts, vector<Vec3> &vnms, vector<rxFace> &plys);

	//! �o�C�i���t�H�[�}�b�g��STL�t�@�C���ǂݍ���
	bool readBinaryData(ifstream &file, vector<Vec3> &vrts, vector<Vec3> &vnms, vector<rxFace> &plys);

	//! ASCII�t�H�[�}�b�g��STL�t�@�C���ǂݍ���
	bool saveAsciiData(ofstream &file, const vector<Vec3> &vrts, const vector<rxFace> &plys);

	//! �o�C�i���t�H�[�}�b�g��STL�t�@�C���ǂݍ���
	bool saveBinaryData(ofstream &file, const vector<Vec3> &vrts, const vector<rxFace> &plys);

};



#endif // _RX_OBJ_H_
