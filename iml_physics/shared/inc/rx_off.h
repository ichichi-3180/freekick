/*!
  @file rx_off.h
	
  @brief OFF File Input
 
  @author Makoto Fujisawa
  @date   2012
*/

#ifndef _RX_OFF_H_
#define _RX_OFF_H_


//-----------------------------------------------------------------------------
// Include Files
//-----------------------------------------------------------------------------
#include "rx_mesh.h"

//-----------------------------------------------------------------------------
// Name Space
//-----------------------------------------------------------------------------
using namespace std;


//-----------------------------------------------------------------------------
// rxOFF�N���X�̐錾 - OFF�`���̓ǂݍ���
//-----------------------------------------------------------------------------
class rxOFF
{
public:
	//! �R���X�g���N�^
	rxOFF();
	//! �f�X�g���N�^
	~rxOFF();

	/*!
	 * OFF�t�@�C���ǂݍ���
	 * @param[in] file_name �t�@�C����(�t���p�X)
	 * @param[out] vrts ���_���W
	 * @param[out] vnms ���_�@��
	 * @param[out] poly �|���S��
	 * @param[in] triangle �|���S���̎O�p�`�����t���O
	 */
	bool Read(string file_name, vector<Vec3> &vrts, vector<Vec3> &vnms, vector<rxFace> &plys, bool triangle = false);

private:
	int loadFace(string &buf, vector<int> &vidxs, vector<int> &nidxs, vector<int> &tidxs);
};



#endif // _RX_OBJ_H_
