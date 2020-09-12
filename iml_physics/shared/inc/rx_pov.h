/*!
  @file rx_pov.h
	
  @brief POV/INC File Output
 
  @author Makoto Fujisawa
  @date   2011
*/
// FILE --rx_pov.h--

#ifndef _RX_POV_H_
#define _RX_POV_H_


//-----------------------------------------------------------------------------
// �C���N���[�h�t�@�C��
//-----------------------------------------------------------------------------
#include "rx_mesh.h"


//-----------------------------------------------------------------------------
// ���O���
//-----------------------------------------------------------------------------
using namespace std;


//-----------------------------------------------------------------------------
// rxPOV�N���X�̐錾 - POV�`���̏o��
//-----------------------------------------------------------------------------
class rxPOV
{
public:
	//! �R���X�g���N�^
	rxPOV(){}

	//! �f�X�g���N�^
	~rxPOV(){}


	/*!
	 * POV-Ray�`���ŕۑ�(rxTriangle)
	 * @param[in] file_name �ۑ��t�@�C����
	 * @param[in] verts ���_���W
	 * @param[in] norms ���_�@��
	 * @param[in] polys �O�p�`���b�V��
	 * @param[in] obj_name POV-Ray�I�u�W�F�N�g��
	 * @param[in] add �ǋL�t���O
	 */
	bool SaveListData(const string &file_name, vector<Vec3> &verts, vector<Vec3> &norms, vector<rxTriangle> &polys, 
					  const string &obj_name = "PolygonObject", bool add = false)
	{
		return SaveListData(file_name, verts, norms, polys, Vec3(0.0), Vec3(1.0), Vec3(0.0), obj_name, add);
	}

	/*!
	 * POV-Ray�`���ŕۑ�(rxTriangle,�ϊ��t��)
	 * @param[in] file_name �ۑ��t�@�C����
	 * @param[in] verts ���_���W
	 * @param[in] norms ���_�@��
	 * @param[in] polys �O�p�`���b�V��
	 * @param[in] trans,scale,rot ���s�ړ��C�X�P�[�����O�C��]
	 * @param[in] obj_name POV-Ray�I�u�W�F�N�g��
	 * @param[in] add �ǋL�t���O
	 */
	bool SaveListData(const string &file_name, vector<Vec3> &verts, vector<Vec3> &norms, vector<rxTriangle> &polys, 
					  const Vec3 &trans, const Vec3 &scale, const Vec3 &rot, 
					  const string &obj_name = "PolygonObject", bool add = false)
	{
		if((int)polys.size() == 0) return false;

		// INC�t�@�C���쐬
		FILE* fp;

		if(add){
			if((fp = fopen(file_name.c_str(),"a")) == NULL) return false;
		}
		else{
			if((fp = fopen(file_name.c_str(),"w")) == NULL) return false;
		}

		fprintf(fp, "#declare %s = mesh2 {\n", obj_name.c_str());

		// ���_�f�[�^�o��
		fprintf(fp, "	vertex_vectors {\n");
		fprintf(fp, "		%d,\n", verts.size());

		int i;
		Vec3 v;
		for(i = 0; i < (int)verts.size(); ++i){
			v = verts[i];
			v *= scale;				// �X�P�[�����O
			v = Rotate3(v, rot);	// ��]
			v += trans;				// ���s�ړ�

			fprintf(fp, "		<%f,%f,%f>\n", v[0], v[1], v[2]);
		}
		fprintf(fp, "	}\n");


		// ���_�@���o��
		fprintf(fp, "	normal_vectors {\n");
		fprintf(fp, "		%d,\n",verts.size());

		Vec3 normal;
		for(i = 0; i < (int)verts.size(); ++i){
			normal = norms[i];
			normal = Rotate3(normal, rot);	// ��]
			fprintf(fp, "		<%f,%f,%f>\n", normal[0], normal[1], normal[2]);
		}
		fprintf(fp, "	}\n");


		// �|���S���C���f�b�N�X�o��
		fprintf(fp, "	face_indices {\n");
	
		int np = (int)polys.size();

		fprintf(fp, "		%d,\n", np);

		for(i = 0; i < np; ++i){
			// �O�p�`�|���S��
			fprintf(fp, "		<%d,%d,%d>,\n", polys[i][0], polys[i][1], polys[i][2]);
		}

	/*
		int j, n, np = 0;
		for(i = 0; i < (int)polys.size(); ++i){
			n = polys[i].size();
			np += n-2;
		}

		fprintf(fp, "		%d,\n", np);

		for(i = 0; i < (int)polys.size(); ++i){
			n = polys[i].size();

			// �O�p�`�|���S���ɕ���
			for(j = 0; j < n-2; ++j){
				fprintf(fp, "		<%d,%d,%d>,\n", polys[i][0], polys[i][j+1], polys[i][j+2]);
			}
		}
	*/
		fprintf(fp, "	}\n");
		fprintf(fp, "	inside_vector <0.0, 0.0, 0.0>\n");
		fprintf(fp, "}   //#declare %s\n\n", obj_name.c_str());

		fclose(fp);

		return true;
	}

	/*!
	 * POV-Ray�`���ŕۑ�(rxFace)
	 * @param[in] file_name �ۑ��t�@�C����
	 * @param[in] verts ���_���W
	 * @param[in] norms ���_�@��
	 * @param[in] polys �|���S�����b�V��
	 * @param[in] obj_name POV-Ray�I�u�W�F�N�g��
	 * @param[in] add �ǋL�t���O
	 */
	bool SaveListData(const string &file_name, vector<Vec3> &verts, vector<Vec3> &norms, vector<rxFace> &polys, 
					  const string &obj_name = "PolygonObject", bool add = false)
	{
		return SaveListData(file_name, verts, norms, polys, Vec3(0.0), Vec3(1.0), Vec3(0.0), obj_name, add);
	}


	/*!
	 * POV-Ray�`���ŕۑ�(rxFace,�ϊ��t��)
	 * @param[in] file_name �ۑ��t�@�C����
	 * @param[in] verts ���_���W
	 * @param[in] norms ���_�@��
	 * @param[in] polys �|���S�����b�V��
	 * @param[in] trans,scale,rot ���s�ړ��C�X�P�[�����O�C��]
	 * @param[in] obj_name POV-Ray�I�u�W�F�N�g��
	 * @param[in] add �ǋL�t���O
	 */
	bool SaveListData(const string &file_name, vector<Vec3> &verts, vector<Vec3> &norms, vector<rxFace> &polys, 
					  const Vec3 &trans, const Vec3 &scale, const Vec3 &rot, 
					  const string &obj_name = "PolygonObject", bool add = false)
	{
		if((int)polys.size() == 0) return false;

		// INC�t�@�C���쐬
		FILE* fp;

		if(add){
			if((fp = fopen(file_name.c_str(),"a")) == NULL) return false;
		}
		else{
			if((fp = fopen(file_name.c_str(),"w")) == NULL) return false;
		}

		fprintf(fp, "#declare %s = mesh2 {\n", obj_name.c_str());

		//
		// ���_�f�[�^�o��
		fprintf(fp, "	vertex_vectors {\n");
		fprintf(fp, "		%d,\n", verts.size());

		int i;
		Vec3 v;
		for(i = 0; i < (int)verts.size(); ++i){
			v = verts[i];
			v *= scale;				// �X�P�[�����O
			v = Rotate3(v, rot);	// ��]
			v += trans;				// ���s�ړ�

			fprintf(fp, "		<%f,%f,%f>\n", v[0], v[1], v[2]);
		}
		fprintf(fp, "	}\n");

		//
		// ���_�@���o��
		fprintf(fp, "	normal_vectors {\n");
		fprintf(fp, "		%d,\n",verts.size());

		Vec3 normal;
		for(i = 0; i < (int)verts.size(); ++i){
			normal = norms[i];
			normal = Rotate3(normal, rot);	// ��]
			fprintf(fp, "		<%f,%f,%f>\n", normal[0], normal[1], normal[2]);
		}
		fprintf(fp, "	}\n");

		//
		// �|���S���C���f�b�N�X�o��
		fprintf(fp, "	face_indices {\n");

		// ���O�p�`���̎Z�o�Əo��
		int np = 0;
		for(i = 0; i < (int)polys.size(); ++i){
			np += polys[i].vert_idx.size()-2;
		}
		fprintf(fp, "		%d,\n", np);

		for(i = 0; i < (int)polys.size(); ++i){
			int n = polys[i].vert_idx.size();

			// �O�p�`�|���S���ɕ���
			for(int j = 0; j < n-2; ++j){
				fprintf(fp, "		<%d,%d,%d>,\n", polys[i][0], polys[i][j+1], polys[i][j+2]);
			}
		}

		fprintf(fp, "	}\n");
		fprintf(fp, "	inside_vector <0.0, 0.0, 0.0>\n");
		fprintf(fp, "}   //#declare %s\n\n", obj_name.c_str());

		fclose(fp);

		return true;
	}

	/*!
	 * POV-Ray�`���ŕۑ�(rxPolygons)
	 * @param[in] file_name �ۑ��t�@�C����
	 * @param[in] polys �|���S���I�u�W�F�N�g
	 * @param[in] obj_name POV-Ray�I�u�W�F�N�g��
	 * @param[in] add �ǋL�t���O
	 */
	bool SaveListData(const string &file_name, rxPolygons &polys, 
					  const string &obj_name = "PolygonObject", bool add = false)
	{
		return SaveListData(file_name, polys.vertices, polys.normals, polys.faces, obj_name, add);
	}

};




#endif // #ifndef _RX_POV_H_