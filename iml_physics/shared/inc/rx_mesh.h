/*! 
  @file rx_mesh.h

  @brief ���b�V���\���̒�`
 
  @author Makoto Fujisawa
  @date 2010
*/
// FILE --rx_mesh.h--

#ifndef _RX_MESH_H_
#define _RX_MESH_H_


//-----------------------------------------------------------------------------
// �C���N���[�h�t�@�C��
//-----------------------------------------------------------------------------
#include <sstream>

#include <string>
#include <vector>
#include <map>
#include <set>

#include <GL/glew.h>
#include <GL/glut.h>

#include "rx_utility.h"

using namespace std;

//-----------------------------------------------------------------------------
// �ގ��N���X
//-----------------------------------------------------------------------------
class rxMaterialOBJ
{
public:
	string name;		//!< �ގ��̖��O

	Vec4 diffuse;		//!< �g�U�F(GL_DIFFUSE)
	Vec4 specular;		//!< ���ʔ��ːF(GL_SPECULAR)
	Vec4 ambient;		//!< �����F(GL_AMBIENT)

	Vec4 color;			//!< �F(glColor)
	Vec4 emission;		//!< ���ːF(GL_EMISSION)

	double shininess;	//!< ���ʔ��ˎw��(GL_SHININESS)

	int illum;
	string tex_file;	//!< �e�N�X�`���t�@�C����
	unsigned int tex_name;	//!< �e�N�X�`���I�u�W�F�N�g
};


typedef map<string, rxMaterialOBJ> rxMTL;


//-----------------------------------------------------------------------------
// ���b�V���N���X
//-----------------------------------------------------------------------------
// �|���S��
class rxFace
{
public:
	vector<int> vert_idx;	//!< ���_�C���f�b�N�X
	string material_name;	//!< �ގ���
	vector<Vec2> texcoords;	//!< �e�N�X�`�����W
	int attribute;			//!< ����

public:
	rxFace() : attribute(0) {}

public:
	// �I�y���[�^�ɂ�钸�_�A�N�Z�X
	inline int& operator[](int i){ return vert_idx[i]; }
	inline int  operator[](int i) const { return vert_idx[i]; }

	// �֐��ɂ�钸�_�A�N�Z�X
	inline int& at(int i){ return vert_idx.at(i); }
	inline int  at(int i) const { return vert_idx.at(i); }

	//! �|���S�����_���̕ύX
	void resize(int size)
	{
		vert_idx.resize(size);
	}

	//! �|���S�����_���̕ύX
	int size(void) const
	{
		return (int)vert_idx.size();
	}

	//! ������
	void clear(void)
	{
		vert_idx.clear();
		material_name = "";
		texcoords.clear();
	}
};

// �O�p�`�|���S��
class rxTriangle : public rxFace
{
public:
	rxTriangle()
	{
		vert_idx.resize(3);
	}
};

// �|���S���I�u�W�F�N�g
class rxPolygons
{
public:
	vector<Vec3> vertices;	//!< ���_���W
	vector<Vec3> normals;	//!< ���_�@��
	vector<rxFace> faces;	//!< �|���S��
	rxMTL materials;		//!< �ގ�
	int open;				//!< �t�@�C���I�[�v���t���O

public:
	//! �R���X�g���N�^
	rxPolygons() : open(0) {}
	//! �f�X�g���N�^
	~rxPolygons(){}

	//! �`��
	void Draw(int draw = 0x04, double dn = 0.02, bool col = true);

protected:
	//! double�ł̍ގ��ݒ�
	void glMaterialdv(GLenum face, GLenum pname, const GLdouble *params);
};



//-----------------------------------------------------------------------------
// ���b�V���f�[�^�̓ǂݍ��݁C�ۑ��ɕ֗��Ȋ֐�
//-----------------------------------------------------------------------------

//! ���W�̉�]
Vec3 Rotate3(const Vec3 &pos0, const Vec3 &rot);

//! "������ ���l"���琔�l�����̕�����݂̂����o��
bool StringToString(const string &buf, const string &head, string &sub);
bool StringToDouble(const string &buf, const string &head, double &val);

//! "x y z"�̌`���̕����񂩂�Vec2�^�֕ϊ�
int StringToVec2s(const string &buf, const string &head, Vec2 &v);

//! "x y z"�̌`���̕����񂩂�Vec3�^�֕ϊ�
int StringToVec3s(const string &buf, const string &head, Vec3 &v);

//! Vec3�^���當����ɕϊ�
string Vec3ToString(const Vec3 v);

//! �����񂩂�x�N�g���l�𒊏o
template<class T> 
inline int ExtractVector(string data, const string &end_str, const string &brk_str, int min_elems, vector< vector<T> > &vecs);

//! �擪�̋�(�X�y�[�X�C�^�u)���폜
string GetDeleteSpace(const string &buf);

//! �擪�̋�(�X�y�[�X�C�^�u)���폜
void DeleteHeadSpace(string &buf);

//! ��(�X�y�[�X�C�^�u)���폜
void DeleteSpace(string &buf);

//! ������Ɋ܂܂��w�肳�ꂽ������̐���Ԃ�
int CountString(string &s, int offset, string c);

//! �|���S�����O�p�`�ɕ���
int PolyToTri(vector<rxFace> &plys, const vector<int> &vidxs, const vector<int> &tidxs, const vector<Vec2> &vtc, string mat_name);

//! �t�@�C��������t�H���_�p�X�݂̂����o��
string ExtractDirPath(const string &fn);

//! �t�@�C��������g���q���폜
string ExtractPathWithoutExt(const string &fn);

//! �����񏬕�����
void StringToLower(string &str);

//! �����񂪐����l��\���Ă��邩�𒲂ׂ�
bool IsInteger(const string &str);

//! �����񂪎����l��\���Ă��邩�𒲂ׂ�
bool IsNumeric(const string &str);

//! ���_�񂩂��AABB�̌���
bool FindBBox(Vec3 &minp, Vec3 &maxp, const vector<Vec3> &vec_set, const int start_index, const int end_index);

//! ���_�񂩂��AABB�̌���
bool FindBBox(Vec3 &minp, Vec3 &maxp, const vector<Vec3> &vec_set);

//! ���_���AABB�ɍ����悤��Fit������
bool FitVertices(const Vec3 &ctr, const Vec3 &sl, vector<Vec3> &vec_set, const int start_index, const int end_index);

//! ���_���AABB�ɍ����悤��Fit������
bool FitVertices(const Vec3 &ctr, const Vec3 &sl, vector<Vec3> &vec_set);

//! �o�C�i���t�@�C������^�w��Ńf�[�^��ǂݍ���
template<class T> 
inline T ReadBinary(ifstream &file);

//! �o�C�i���t�@�C������^�E�T�C�Y�w��Ńf�[�^��ǂݍ���
template<class T> 
inline T ReadBinary(ifstream &file, int byte);

//! �o�C�i���t�@�C������2�o�C�g���ǂݍ����int�^�Ɋi�[
inline int ReadInt(ifstream &file);

//! �o�C�i���t�@�C�����當�����ǂݎ��
inline bool ReadString(ifstream &file, string &name, int max_size = -1);


//-----------------------------------------------------------------------------
// Geometry processing
//-----------------------------------------------------------------------------
//! �@�����]
void ReverseNormals(const vector<Vec3> &vs, vector<int> &ps, vector<Vec3> &ns);

//! ���b�V���@���v�Z
void CalNormals(const vector<Vec3> &vs, vector<int> &ps, vector<Vec3> &ns);

//! ���_�@���v�Z
void CalVertexNormals(const vector<Vec3> &vrts, int nvrts, vector<rxTriangle> &tris, int ntris, vector<Vec3> &vnrms);
void CalVertexNormals(const vector<Vec3> &vrts, int nvrts, vector<rxFace> &tris, int ntris, vector<Vec3> &vnrms);

//! ���_�@���v�Z
void CalVertexNormals(rxPolygons &polys);

//! ���_�@���v�Z
// void CalVertexNormalsFromVBO(GLuint vrts_vbo, GLuint tris_vbo, GLuint nrms_vbo, uint nvrts, uint ntris);

//! �􉽏��������Ȃ��|���S�����_��ƃ|���S���@�����璸�_�@�����v�Z
void CalVertexNormalWithoutGeometry(const vector<Vec3> &vrts, vector<Vec3> &nrms);

//! �􉽏��������Ȃ��|���S�����_�񂩂�􉽏��𐶐�
void CalVertexGeometry(vector<Vec3> &vrts, vector< vector<int> > &idxs);

//! ���_���AABB�ɍ����悤��Fit������
bool AffineVertices(rxPolygons &polys, Vec3 cen, Vec3 ext, Vec3 ang);

//! �V�~�����[�V������Ԃ𕢂��O���b�h�̎Z�o
int CalMeshDiv(Vec3 &minp, Vec3 maxp, int nmax, double &h, int n[3], double extend = 0.05);

//! ���b�V���`��p��VBO���m��
bool AssignArrayBuffers(int max_verts, int dim, GLuint &uVrtVBO, GLuint &uNrmVBO, GLuint &uTriVBO);

//! FBO�Ƀf�[�^��ݒ�
bool SetArrayFromFBO(GLuint uVrtVBO, GLuint uNrmVBO, GLuint uTriVBO, 
					 vector<Vec3> &vrts, vector<Vec3> &nrms, vector<rxFace> &face, int nvrts, int ntris);

//! �z�X�g���z��Ƀf�[�^��ݒ�
bool SetFBOFromArray(GLuint uVrtVBO, GLuint uNrmVBO, GLuint uTriVBO, 
					 vector<Vec3> &vrts, vector<Vec3> &nrms, vector<rxFace> &face);




//-----------------------------------------------------------------------------
// MARK:�|���S���̕`��
//-----------------------------------------------------------------------------
inline void rxPolygons::glMaterialdv(GLenum face, GLenum pname, const GLdouble *params)
{
	GLfloat col[4];
	col[0] = (GLfloat)params[0];
	col[1] = (GLfloat)params[1];
	col[2] = (GLfloat)params[2];
	col[3] = (GLfloat)params[3];
	glMaterialfv(face, pname, col);
}

/*!
 * �|���S���̕`��
 * @param[in] polys �|���S���f�[�^
 * @param[in] draw �`��t���O(���ʃr�b�g���璸�_,�G�b�W,��,�@�� - 1,2,4,8)
 */
inline void rxPolygons::Draw(int draw, double dn, bool col)
{
	// ���_���ƃ|���S����
	int vn = (int)vertices.size();
	int pn = (int)faces.size();
		
	if(draw & 0x02){
		// �G�b�W�`��ɂ�����"stitching"���Ȃ������߂̃I�t�Z�b�g�̐ݒ�
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0, 1.0);
	}

	if(draw & 0x04){
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glEnable(GL_LIGHTING);
		if(col) glColor3d(0.0, 0.0, 1.0);
		bool use_mat = true;
		if(materials.empty()){
			// �ގ����ݒ肳��Ă��Ȃ��C�������́C�G�b�W�̂ݕ`��̏ꍇ�CGL_COLOR_MATERIAL��p����
			use_mat = false;

			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glEnable(GL_COLOR_MATERIAL);
		}
		else{
			glDisable(GL_COLOR_MATERIAL);
		}


		bool tc = false;
		rxMaterialOBJ *cur_mat = NULL, *pre_mat = NULL;

		// ���ׂẴ|���S����`��
		for(int i = 0; i < pn; ++i){
			rxFace *face = &(faces[i]);
			int n = (int)face->vert_idx.size();

			if(use_mat){
				// �ގ��̐ݒ�
				cur_mat = &materials[face->material_name];

				// ���݂̍ގ����O�̂ƈقȂ�ꍇ�̂݁COpenGL�̍ގ������X�V
				if(cur_mat != pre_mat){
					glColor4dv(cur_mat->color);
					glMaterialdv(GL_FRONT_AND_BACK, GL_DIFFUSE,  cur_mat->diffuse.data);
					glMaterialdv(GL_FRONT_AND_BACK, GL_SPECULAR, cur_mat->specular.data);
					glMaterialdv(GL_FRONT_AND_BACK, GL_AMBIENT,  cur_mat->ambient.data);
					glMaterialdv(GL_FRONT_AND_BACK, GL_EMISSION, cur_mat->emission.data);
					glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, (GLfloat)cur_mat->shininess);

					// �e�N�X�`��������΃o�C���h
					if(cur_mat->tex_name){
						glEnable(GL_TEXTURE_2D);
						glBindTexture(GL_TEXTURE_2D, cur_mat->tex_name);
						glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
					}
					else{
						glBindTexture(GL_TEXTURE_2D, 0);
						glDisable(GL_TEXTURE);
					}

					pre_mat = cur_mat;
				}
			}

			// �|���S���`��
			glBegin(GL_POLYGON);
			for(int j = 0; j < n; ++j){
				int idx = face->vert_idx[j];
				if(idx >= 0 && idx < vn){
					glNormal3dv(normals[idx].data);
					if(!face->texcoords.empty()) glTexCoord2dv(face->texcoords[j].data);
					glVertex3dv((vertices[idx]).data);
				}
			}
			glEnd();
		}

		glBindTexture(GL_TEXTURE_2D, 0);
		glDisable(GL_TEXTURE_2D);
	}


	// ���_�`��
	if(draw & 0x01){
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0, 1.0);

		glDisable(GL_LIGHTING);
		if(col) glColor3d(1.0, 0.0, 0.0);
		glPointSize(5.0);
		glBegin(GL_POINTS);
		for(int i = 0; i < vn; ++i){
			glVertex3dv(vertices[i].data);
		}
		glEnd();
	}

	// �G�b�W�`��
	if(draw & 0x02){
		glDisable(GL_LIGHTING);
		if(col) glColor3d(0.3, 1.0, 0.0);
		glLineWidth(1.0);
		for(int i = 0; i < pn; ++i){
			rxFace *face = &faces[i];
			int n = (int)face->vert_idx.size();
			
			glBegin(GL_LINE_LOOP);
			for(int j = 0; j < n; ++j){
				int idx = face->vert_idx[j];
				if(idx >= 0 && idx < vn){
					glVertex3dv(vertices[idx].data);
				}
			}
			glEnd();
		}
	}
	//glDisable(GL_POLYGON_OFFSET_FILL);

	// �@���`��
	if(draw & 0x08){
		glDisable(GL_LIGHTING);
		glColor3d(0.0, 0.9, 0.0);
		glLineWidth(1.0);
		for(int i = 0; i < vn; ++i){
			glBegin(GL_LINES);
			glVertex3dv(vertices[i].data);
			glVertex3dv((vertices[i]+dn*normals[i]).data);
			glEnd();
		}
	}
}


//-----------------------------------------------------------------------------
// MARK:�֐��̎���
//-----------------------------------------------------------------------------

/*!
 * ���W�̉�]
 * @param[in] pos0 ��]���������W�l
 * @param[in] rot0 x,y,z������̉�]�p(deg)
 * @return �J�X��̍��W�l
 */
inline Vec3 Rotate3(const Vec3 &pos0, const Vec3 &rot)
{
	Vec3 S, C;
	S = Vec3(sin(rot[0]), sin(rot[1]), sin(rot[2]));
	C = Vec3(cos(rot[0]), cos(rot[1]), cos(rot[2]));

	Vec3 pos1;
	pos1[0] = C[1]*C[2]*pos0[0]-C[1]*S[2]*pos0[1]+S[1]*pos0[2];
	pos1[1] = (S[0]*S[1]*C[2]+C[0]*S[2])*pos0[0]+(-S[0]*S[1]*S[2]+C[0]*C[2])*pos0[1]-S[0]*C[1]*pos0[2];
	pos1[2] = (-C[0]*S[1]*C[2]+S[0]*S[2])*pos0[0]+(C[0]*S[1]*S[2]+S[0]*C[2])*pos0[1]+C[0]*C[1]*pos0[2];
	return pos1;
}

/*!
 * "������ ���l"���琔�l�����̕�����݂̂����o��
 * @param[in] buf ���̕�����
 * @param[in] head ��������
 * @param[out] sub ���l�����̕�����
 */
inline bool StringToString(const string &buf, const string &head, string &sub)
{
	size_t pos = 0;
	if((pos = buf.find(head)) == string::npos) return false;
	pos += head.size();

	if((pos = buf.find_first_not_of(" �@\t", pos)) == string::npos) return false;
	
	sub = buf.substr(pos);
	return true;
}

/*!
 * "������ ���l"���琔�l�����݂̂����o��
 * @param[in] buf ���̕�����
 * @param[in] head ��������
 * @param[out] val ���l
 */
inline bool StringToDouble(const string &buf, const string &head, double &val)
{
	string sub;
	if(StringToString(buf, head, sub)){
		sscanf(&sub[0], "%lf", &val);
		return true;
	}
	else{
		return false;
	}
}

/*!
 * "������ ���l"���琔�l�����݂̂����o��
 * @param[in] buf ���̕�����
 * @param[in] head ��������
 * @param[out] val ���l
 */
inline bool StringToInt(const string &buf, const string &head, int &val)
{
	string sub;
	if(StringToString(buf, head, sub)){
		sscanf(&sub[0], "%d", &val);
		return true;
	}
	else{
		return false;
	}
}


/*!
 * "x y z"�̌`���̕����񂩂�Vec2�^�֕ϊ�
 * @param[in] s ������
 * @param[out] v �l
 * @return �v�f�L�q��
 */
inline int StringToVec2s(const string &buf, const string &head, Vec2 &v)
{
	string sub;
	StringToString(buf, head, sub);

	Vec2 tmp;

	if(sscanf(&sub[0], "%lf %lf", &tmp[0], &tmp[1]) != 2){
		return 0;
	}
	v = tmp;

	return 2;
}

/*!
 * "x y z"�̌`���̕����񂩂�Vec3�^�֕ϊ�
 * @param[in] s ������
 * @param[out] v �l
 * @return �v�f�L�q��
 */
inline int StringToVec3s(const string &buf, const string &head, Vec3 &v)
{
	string sub;
	StringToString(buf, head, sub);

	Vec3 tmp;
	double tmp_w;

	if(sscanf(&sub[0], "%lf %lf %lf %lf", &tmp[0], &tmp[1], &tmp[2], &tmp_w) != 4){
		if(sscanf(&sub[0], "%lf %lf %lf", &tmp[0], &tmp[1], &tmp[2]) != 3){
			return 0;
		}
	}
	v = tmp;

	return 3;
}

inline string Vec3ToString(const Vec3 v)
{
	stringstream ss;
	ss << v[0] << " " << v[1] << " " << v[2];
	return ss.str();
	//string s;
	//sscanf(&s[0], "%f %f %f", v[0], v[1], v[2]);
	//return s;
}

/*!
 * �����񂩂�x�N�g���l�𒊏o
 *  - ���Ƃ��΁C"0, 1, 2"���ƁC"��end_str, ","��brk_str
 * @param[in] data ���̕�����
 * @param[in] end_str �x�N�g���̋�؂蕶��
 * @param[in] brk_str �x�N�g���v�f�Ԃ̋�؂蕶��
 * @param[in] min_elem �ŏ��v�f��
 * @param[out] vecs �x�N�g���l
 * @return ���������x�N�g���̐�
 */
template<class T> 
inline int ExtractVector(string data, const string &end_str, const string &brk_str, int min_elems, 
						 vector< vector<T> > &vecs)
{
	data += end_str;	// ��̏����̂��߂ɋ�؂蕶������Ō�ɑ����Ă���
	int n = 0;
	size_t cpos[2] = {0, 0};
	while((cpos[1] = data.find(end_str, cpos[0])) != string::npos){
		// ��؂蕶��������������C�O��̋�؂蕶���ʒu�Ƃ̊Ԃ̕�����𔲂��o��
		string sub = data.substr(cpos[0], cpos[1]-cpos[0]);
		if(sub.empty()){
			cpos[0] = cpos[1]+end_str.size();
			break;
		}

		// �����o������������e�x�N�g���v�f�ɕ���
		sub += brk_str;
		vector<T> val;
		size_t spos[2] = {0, 0};
		while((spos[1] = sub.find(brk_str, spos[0])) != string::npos){
			string val_str = sub.substr(spos[0], spos[1]-spos[0]);
			DeleteSpace(val_str);
			if(val_str.empty()){
				spos[0] = spos[1]+brk_str.size();
				continue;
			}

			val.push_back((T)atof(val_str.c_str()));
			spos[0] = spos[1]+brk_str.size();
		}
		if((int)val.size() >= min_elems){
			vecs.push_back(val);
			n++;
		}
		cpos[0] = cpos[1]+end_str.size();
	}

	return n;
}

/*!
 * �����񂩂琔�l��𒊏o
 * @param[in] data ���̕�����
 * @param[in] brk_str �v�f�Ԃ̋�؂蕶��
 * @param[out] strs �e���l(string)
 * @return ���������v�f�̐�
 */
inline int ExtractSubStr(string data, const string &brk_str, vector<string> &strs)
{
	data += brk_str;	// ��̏����̂��߂ɋ�؂蕶������Ō�ɑ����Ă���
	int n = 0;
	size_t cpos[2] = {0, 0};
	while((cpos[1] = data.find(brk_str, cpos[0])) != string::npos){
		// ��؂蕶��������������C�O��̋�؂蕶���ʒu(cpos[0])�Ƃ̊Ԃ̕�����𔲂��o��
		string sub = data.substr(cpos[0], cpos[1]-cpos[0]);
		if(!sub.empty()){
			strs.push_back(sub);
			n++;
		}
		
		cpos[0] = cpos[1]+brk_str.size();
	}

	return n;
}

/*!
 * �擪�̋�(�X�y�[�X�C�^�u)���폜
 * @param[in] buf ���̕�����
 * @return �󔒍폜��̕�����
 */
inline string GetDeleteSpace(const string &buf)
{
	string buf1 = buf;

	size_t pos;
	while((pos = buf1.find_first_of(" �@\t")) == 0){
		buf1.erase(buf1.begin());
		if(buf1.empty()) break;
	}

	return buf1;
}

/*!
 * �擪�̋�(�X�y�[�X�C�^�u)���폜
 * @param[inout] buf ����������
 */
inline void DeleteHeadSpace(string &buf)
{
	size_t pos;
	while((pos = buf.find_first_of(" �@\t")) == 0){
		buf.erase(buf.begin());
		if(buf.empty()) break;
	}
}

/*!
 * ��(�X�y�[�X�C�^�u)���폜
 * @param[inout] buf ����������
 */
inline void DeleteSpace(string &buf)
{
	size_t pos;
	while((pos = buf.find_first_of(" �@\t")) != string::npos){
		buf.erase(pos, 1);
	}
}

/*!
 * ������Ɋ܂܂��w�肳�ꂽ������̐���Ԃ�
 * @param[in] s ���̕�����
 * @param[in] c ����������
 * @return �܂܂�鐔
 */
inline int CountString(string &s, int offset, string c)
{
	int count = 0;
	size_t pos0 = offset, pos = 0;
	int n = (int)c.size();

	while((pos = s.find(c, pos0)) != string::npos){
		if(pos != pos0){
			count++;
		}
		else{
			s.erase(s.begin()+pos);
		}
		pos0 = pos+n;
	}

	// �Ō�̕����񏜋�
	if(s.rfind(c) == s.size()-n){
		count--;
	}

	return count;
}

/*!
 * �|���S�����O�p�`�ɕ���
 * @param[out] ply �O�p�`�ɕ������ꂽ�|���S����
 * @param[in] vidx �|���S��(���_��4�ȏ�)�̒��_�C���f�b�N�X
 * @param[in] mat_name �ގ���
 * @return �������ꂽ�O�p�`��
 */
inline int PolyToTri(vector<rxFace> &plys, 
					 const vector<int> &vidxs, 
					 string mat_name)
{
	int n = (int)vidxs.size();

	if(n <= 3) return 0;

	rxFace face;
	face.resize(3);
	face.material_name = mat_name;

	int num_tri = n-2;
	for(int i = 0; i < num_tri; ++i){
		face[0] = vidxs[0];
		face[1] = vidxs[i+1];
		face[2] = vidxs[i+2];

		if(face[1] != face[2] && face[0] != face[1] && face[2] != face[0]){
			plys.push_back(face);
		}
	}

	return num_tri;
}

/*!
 * �|���S�����O�p�`�ɕ���(with �e�N�X�`�����W)
 * @param[out] ply �O�p�`�ɕ������ꂽ�|���S����
 * @param[in] vidx �|���S��(���_��4�ȏ�)�̒��_�C���f�b�N�X
 * @param[in] tidx �|���S��(���_��4�ȏ�)�̃e�N�X�`�����W�C���f�b�N�X
 * @param[in] vtc  �e�N�X�`�����W�x�N�g��
 * @param[in] mat_name �ގ���
 * @return �������ꂽ�O�p�`��
 */
inline int PolyToTri(vector<rxFace> &plys, 
					 const vector<int> &vidxs, 
					 const vector<int> &tidxs, 
					 const vector<Vec2> &vtc, 
					 string mat_name)
{
	int n = (int)vidxs.size();

	if(n <= 3) return 0;

	rxFace face;
	face.vert_idx.resize(3);
	face.material_name = mat_name;
	face.texcoords.resize(3);

	bool tc = !vtc.empty();

	int num_tri = n-2;
	for(int i = 0; i < num_tri; ++i){
		face[0] = vidxs[0];
		face[1] = vidxs[i+1];
		face[2] = vidxs[i+2];

		if(tc){
			face.texcoords[0] = vtc[tidxs[0]];
			face.texcoords[1] = vtc[tidxs[i+1]];
			face.texcoords[2] = vtc[tidxs[i+2]];
		}
		else{
			face.texcoords[0] = Vec2(0.0);
			face.texcoords[1] = Vec2(0.0);
			face.texcoords[2] = Vec2(0.0);
		}

		plys.push_back(face);
	}

	return num_tri;
}


/*!
 * �t�@�C��������t�H���_�p�X�݂̂����o��
 * @param[in] fn �t�@�C����(�t���p�X or ���΃p�X)
 * @return �t�H���_�p�X
 */
inline string ExtractDirPath(const string &fn)
{
	string::size_type pos;
	if((pos = fn.find_last_of("/")) == string::npos){
		if((pos = fn.find_last_of("\\")) == string::npos){
			return "";
		}
	}

	return fn.substr(0, pos);
}

/*!
 * �t�@�C��������g���q���폜
 * @param[in] fn �t�@�C����(�t���p�X or ���΃p�X)
 * @return �t�H���_�p�X
 */
inline string ExtractPathWithoutExt(const string &fn)
{
	string::size_type pos;
	if((pos = fn.find_last_of(".")) == string::npos){
		return fn;
	}

	return fn.substr(0, pos);
}


/*!
 * �����񏬕�����
 * @param[inout] str ������
 */
inline void StringToLower(string &str)
{
	string::size_type i, size;

	size = str.size();

	for(i = 0; i < size; i++){
		if(str[i] >= 'A' && str[i] <= 'Z') str[i] += 32;
	}

	return;
}

/*!
 * �����񂪐����l��\���Ă��邩�𒲂ׂ�
 * @param[inout] str ������
 * @return �����l�Ȃ�true
 */
inline bool IsInteger(const string &str)
{
	if(str.find_first_not_of("-0123456789 \t") != string::npos) {
		return false;
	}

	return true;
}

/*!
 * �����񂪎����l��\���Ă��邩�𒲂ׂ�
 * @param[inout] str ������
 * @return �����l�Ȃ�true
 */
inline bool IsNumeric(const string &str)
{
	if(str.find_first_not_of("-0123456789. Ee\t") != string::npos) {
		return false;
	}

	return true;
}


/*!
 * �o�C�i���t�@�C������^�w��Ńf�[�^��ǂݍ���
 * @param[inout] file �t�@�C�����̓X�g���[��
 * @return �ǂݍ��񂾐��l
 */
template<class T> 
inline T ReadBinary(ifstream &file)
{
	T data;
	file.read((char*)&data, sizeof(T));
	return data;
}

/*!
 * �o�C�i���t�@�C������^�E�T�C�Y�w��Ńf�[�^��ǂݍ���
 * @param[inout] file �t�@�C�����̓X�g���[��
 * @param[inout] byte �ǂݍ��ރo�C�g��
 * @return �ǂݍ��񂾐��l
 */
template<class T> 
inline T ReadBinary(ifstream &file, int byte)
{
	T data = 0;
	file.read((char*)&data, byte);
	return data;
}

/*!
 * �o�C�i���t�@�C������2�o�C�g���ǂݍ����int�^�Ɋi�[
 * @param[inout] file �t�@�C�����̓X�g���[��
 * @return �ǂݍ��񂾐��l
 */
inline int ReadInt(ifstream &file)
{
	int data = 0;
	file.read((char*)&data, 2);
	return data;
}

/*!
 * �o�C�i���t�@�C�����當�����ǂݎ��
 * @param[inout] file �t�@�C�����̓X�g���[��
 * @param[out] name ������(�����Ȃ���΋�)
 * @param[in] max_size �ǂݍ��ޕ�����(-1�Ȃ�\0�܂œǂݍ���)
 * @return �����񂪂Ȃ�(�X�g���[���̍ŏ���\0)�Ȃ�false��Ԃ�
 */
inline bool ReadString(ifstream &file, string &name, int max_size)
{
	char c = ReadBinary<char>(file);
	if((int)c == 0) return false;

	name.clear();
	name.push_back(c);
	do{
		c = ReadBinary<char>(file);
		name.push_back(c);
	}while(((int)c != 0) && (max_size == -1 || (int)name.size() < max_size));
	name.push_back((char)0);

	return true;
}


//-----------------------------------------------------------------------------
// MARK:Geometry processing
//-----------------------------------------------------------------------------
/*!
 * �@�����]
 * @param[in] vs ���_���X�g
 * @param[inout] ps ���b�V�����X�g
 * @param[inout] ns �@�����X�g
 */
static void ReverseNormals(const vector<Vec3> &vs, vector<int> &ps, vector<Vec3> &ns)
{
	if(ps.empty()) return;

	int nvp = 3;
	int pn = (int)ps.size()/nvp;

	vector<int> idx(nvp);
	for(int i = 0; i < pn; ++i){
		for(int j = 0; j < nvp; ++j){
			idx[j] = ps[nvp*i+j];
		}

		for(int j = 0; j < nvp; ++j){
			ps[nvp*i+j] = idx[nvp-j-1];
		}
	}

	if(ns.empty() || (int)ns.size() != pn){
		// �@���Čv�Z
		ns.resize(pn);
		for(int i = 0; i < pn; ++i){
			ns[i] = Unit(cross(vs[ps[nvp*i+nvp-1]]-vs[ps[nvp*i+0]], vs[ps[nvp*i+1]]-vs[ps[nvp*i+0]]));
		}
	}
	else{
		// �@�����]
		for(int i = 0; i < pn; ++i){
			ns[i] = -ns[i];
		}
	}
}

/*!
 * ���b�V���@���v�Z
 * @param[in] vs ���_���X�g
 * @param[inout] ps ���b�V�����X�g
 * @param[inout] ns �@�����X�g
 */
static void CalNormals(const vector<Vec3> &vs, vector<int> &ps, vector<Vec3> &ns)
{
	if(ps.empty()) return;

	int nvp = 3;
	int pn = (int)ps.size()/nvp;
	if(ns.empty() || (int)ns.size() != pn){
		// �@���Čv�Z
		ns.resize(pn);
	}

	for(int i = 0; i < pn; ++i){
		int *idx = &ps[nvp*i];
		ns[i] = Unit(cross(vs[idx[1]]-vs[idx[0]], vs[idx[nvp-1]]-vs[idx[0]]));
		//ns[i] *= -1;
	}
}


/*!
 * ���_�@���v�Z
 * @param[in] vrts ���_���W
 * @param[in] nvrts ���_��
 * @param[in] tris �O�p�`�|���S���􉽏��
 * @param[in] ntris �O�p�`�|���S����
 * @param[out] nrms �@��
 * @param[out] nnrms �@����(=���_��)
 */
static void CalVertexNormals(const vector<Vec3> &vrts, int nvrts, vector<rxTriangle> &tris, int ntris, 
							 vector<Vec3> &vnrms)
{
	int nnrms = nvrts;
	vnrms.resize(nnrms);
	
	// ���_�@���̏�����
	for(int i = 0; i < nnrms; i++){
		vnrms[i][0] = 0;
		vnrms[i][1] = 0;
		vnrms[i][2] = 0;
	}

	// �@���v�Z
	for(int i = 0; i < ntris; i++){
		Vec3 edge_vec1, edge_vec2, face_normal;

		// �ʖ@�����v�Z
		edge_vec1 = vrts[tris[i][1]]-vrts[tris[i][0]];
		edge_vec2 = vrts[tris[i][2]]-vrts[tris[i][0]];
		face_normal = Unit(cross(edge_vec1, edge_vec2));

		// �|���S���ɏ������钸�_�̖@���ɐώZ
		vnrms[tris[i][0]] += face_normal;
		vnrms[tris[i][1]] += face_normal;
		vnrms[tris[i][2]] += face_normal;
	}

	// ���_�@���𐳋K��
	for(int i = 0; i < nnrms; i++){
		normalize(vnrms[i]);
	}

}

/*!
 * ���_�@���v�Z
 * @param[in] vrts ���_���W
 * @param[in] nvrts ���_��
 * @param[in] tris �O�p�`�|���S���􉽏��
 * @param[in] ntris �O�p�`�|���S����
 * @param[out] nrms �@��
 * @param[out] nnrms �@����(=���_��)
 */
static void CalVertexNormals(const vector<Vec3> &vrts, int nvrts, vector<rxFace> &tris, int ntris, 
							 vector<Vec3> &vnrms)
{
	int nnrms = nvrts;
	vnrms.resize(nnrms);
	
	// ���_�@���̏�����
	for(int i = 0; i < nnrms; i++){
		vnrms[i][0] = 0;
		vnrms[i][1] = 0;
		vnrms[i][2] = 0;
	}

	// �@���v�Z
	for(int i = 0; i < ntris; i++){
		Vec3 edge_vec1, edge_vec2, face_normal;
		int n = (int)tris[i].size();

		// �ʖ@�����v�Z
		edge_vec1 = vrts[tris[i][1]]-vrts[tris[i][0]];
		edge_vec2 = vrts[tris[i][n-1]]-vrts[tris[i][0]];
		face_normal = Unit(cross(edge_vec1, edge_vec2));

		// �|���S���ɏ������钸�_�̖@���ɐώZ
		for(int j = 0; j < n; ++j){
			vnrms[tris[i][j]] += face_normal;
		}
	}

	// ���_�@���𐳋K��
	for(int i = 0; i < nnrms; i++){
		normalize(vnrms[i]);
	}
}
/*!
 * ���_�@���v�Z
 * @param[in] polys �|���S��
 */
static void CalVertexNormals(rxPolygons &polys)
{
	//(const vector<Vec3> &vrts, uint nvrts, vector<rxTriangle> &tris, uint ntris, vector<Vec3> &nrms)
	int pn = (int)polys.faces.size();
	int vn = (int)polys.vertices.size();

	vector<Vec3> fnrms;
	fnrms.resize(pn);

	int max_n = 0;

	// �ʖ@���̌v�Z
	for(int i = 0; i < pn; ++i){
		int n = (int)polys.faces[i].vert_idx.size()-1;
		fnrms[i] = Unit(cross(polys.vertices[polys.faces[i][1]]-polys.vertices[polys.faces[i][0]], 
							  polys.vertices[polys.faces[i][n]]-polys.vertices[polys.faces[i][0]]));

		n = n+1;
		if(n > max_n) max_n = n;
	}

	polys.normals.clear();
	polys.normals.resize(vn);
	for(int i = 0; i < vn; ++i){
		polys.normals[i] = Vec3(0.0);
	}

	// ���_�@���̌v�Z
	vector<int> id;
	id.resize(max_n);
	for(int i = 0; i < pn; ++i){
		int n = (int)polys.faces[i].vert_idx.size();
		for(int j = 0; j < n; ++j){
			polys.normals[polys.faces[i][j]] += fnrms[i];
		}
	}

	// �@�����K��
	for(int i = 0; i < vn; i++){
		normalize(polys.normals[i]);
	}
}


/*!
 * �􉽏��������Ȃ��|���S�����_��ƃ|���S���@�����璸�_�@�����v�Z
 * @param[in] vrts �􉽏�񖳂��̃|���S�����_��
 * @param[inout] nrms �|���S���@�������ꂼ��̒��_�@���Ƃ��Ċi�[����Ă���
 */
static void CalVertexNormalWithoutGeometry(const vector<Vec3> &vrts, vector<Vec3> &nrms)
{
	int n = (int)vrts.size();

	int same_vrts[256];

	vector<int> find_vrts;
	find_vrts.resize(n);
	for(int i = 0; i < n; ++i){
		find_vrts[i] = 0;
	}

	int nvrts = 0;
	int avg_cnt = 0;

	double eps2 = 1e-4;
	for(int i = 0; i < n; ++i){
		if(find_vrts[i]) continue;
		
		Vec3 pos0 = vrts[i];
		Vec3 nrm0 = nrms[i];
		same_vrts[0] = i;
		int cnt = 1;
		find_vrts[i] = 1;
		for(int j = 0; j < n; ++j){
			if(find_vrts[j]) continue;

			Vec3 pos1 = vrts[j];

			if(norm2(pos0-pos1) < eps2){
				find_vrts[j] = 1;
				nrm0 += nrms[j];
				same_vrts[cnt] = j;
				cnt++;
			}
		}

		nrm0 /= (double)cnt;

		for(int j = 0; j < cnt; ++j){
			nrms[same_vrts[j]] = nrm0;
		}

		avg_cnt += cnt;
		nvrts++;
	}

	avg_cnt /= nvrts;
	//cout << avg_cnt << ", " << nvrts << endl;
}


/*!
 * �􉽏��������Ȃ��|���S�����_�񂩂�􉽏��𐶐�
 * @param[inout] vrts �􉽏�񖳂��̃|���S�����_��
 * @param[out] idxs �􉽏��
 */
static void CalVertexGeometry(vector<Vec3> &vrts, vector< vector<int> > &idxs)
{
	int nv = (int)vrts.size();
	int np = nv/3;	// �O�p�`�|���S����

	// �􉽏��
	idxs.resize(np);
	for(int i = 0; i < np; ++i) idxs[i].resize(3);

	int same_vrts[256];	// �d�����_�̃C���f�b�N�X���i�[

	vector<Vec3> compacted_vrts;	// �d���Ȃ����_����i�[����R���e�i

	// �d�����������ς݃t���O�i�[�R���e�i�̊m�ۂƏ�����
	vector<int> find_vrts;
	find_vrts.resize(nv);
	for(int i = 0; i < nv; ++i){
		find_vrts[i] = 0;
	}

	int nvrts = 0;
	int avg_cnt = 0;

	double eps2 = 1e-4;
	for(int i = 0; i < nv; ++i){
		if(find_vrts[i]) continue;
		
		Vec3 pos0 = vrts[i];

		// �܂��d����������Ă��Ȃ����_���i�[
		compacted_vrts.push_back(pos0);
		idxs[i/3][i%3] = nvrts;

		same_vrts[0] = i;
		int cnt = 1;
		find_vrts[i] = 1;
		for(int j = 0; j < nv; ++j){
			if(find_vrts[j]) continue;

			Vec3 pos1 = vrts[j];

			// �������߂��_���d�����_�Ƃ���
			if(norm2(pos0-pos1) < eps2){
				find_vrts[j] = 1;
				same_vrts[cnt] = j;

				idxs[j/3][j%3] = nvrts;	// compacted_vrts���̃C���f�b�N�X

				cnt++;	// �d�������J�E���g
			}
		}

		avg_cnt += cnt;
		nvrts++;
	}

	avg_cnt /= nvrts;

	vrts = compacted_vrts;
}

/*!
 * �I�C���[�p�����]�s��ɕϊ�
 * @param[in] ang �I�C���[�p
 * @param[out] mat ��]�s��
 */
inline void EulerToMatrix(Vec3 ang, double mat[9])
{
	ang[1] = RX_TO_RADIANS(ang[1]);
	ang[0] = RX_TO_RADIANS(ang[0]);
	ang[2] = RX_TO_RADIANS(ang[2]);

	double cy = cos(ang[1]); 
	double sy = sin(ang[1]); 
	double cp = cos(ang[0]); 
	double sp = sin(ang[0]); 
	double cr = cos(ang[2]);
	double sr = sin(ang[2]);

	double cc = cy*cr; 
	double cs = cy*sr; 
	double sc = sy*cr; 
	double ss = sy*sr;

	mat[0] = cc+sp*ss;
	mat[1] = cs-sp*sc;
	mat[2] = -sy*cp;

	mat[3] = -cp*sr;
	mat[4] = cp*cr;
	mat[5] = -sp;

	mat[6] = sc-sp*cs;
	mat[7] = ss+sp*cc;
	mat[8] = cy*cp;
}

/*!
 * �I�C���[�p�����]�s��ɕϊ�
 * @param[in] ang �I�C���[�p
 * @param[out] mat ��]�s��
 */
inline void EulerToMatrix16(Vec3 ang, double mat[16])
{
	ang[1] = RX_TO_RADIANS(ang[1]);
	ang[0] = RX_TO_RADIANS(ang[0]);
	ang[2] = RX_TO_RADIANS(ang[2]);

	double cy = cos(ang[1]); 
	double sy = sin(ang[1]); 
	double cp = cos(ang[0]); 
	double sp = sin(ang[0]); 
	double cr = cos(ang[2]);
	double sr = sin(ang[2]);

	double cc = cy*cr; 
	double cs = cy*sr; 
	double sc = sy*cr; 
	double ss = sy*sr;

	mat[0]  = cc+sp*ss;
	mat[1]  = cs-sp*sc;
	mat[2]  = -sy*cp;

	mat[4]  = -cp*sr;
	mat[5]  = cp*cr;
	mat[6]  = -sp;

	mat[8]  = sc-sp*cs;
	mat[9]  = ss+sp*cc;
	mat[10] = cy*cp;
}

/*!
 * ���_���AABB�ɍ����悤��Fit������(��]�L��C�A�X�y�N�g�䖳��)
 * @param[inout] poly �|���S��
 * @param[in] cen AABB���S���W
 * @param[in] ext AABB�̕ӂ̒���(1/2)
 * @param[in] ang ��]�x�N�g��
 */
static bool AffineVertices(rxPolygons &polys, Vec3 cen, Vec3 ext, Vec3 ang)
{
	int vn = (int)polys.vertices.size();
	if(vn <= 1) return false;

	// ���݂�BBox�̑傫���𒲂ׂ�
	Vec3 minp, maxp;
	minp = maxp = polys.vertices[0];
	for(int i = 1; i < vn; ++i){
		Vec3 pos = polys.vertices[i];
		for(int i = 0; i < 3; ++i){
			if(pos[i] > maxp[i]) maxp[i] = pos[i];
			if(pos[i] < minp[i]) minp[i] = pos[i];
		}
	}
	
	Vec3 scale = (maxp-minp);
	Vec3 trans = (maxp+minp)/2.0;

	for(int i = 0; i < 3; ++i){
		if(fabs(scale[i]) < RX_FEQ_EPS){
			scale[i] = 1.0;
		}
	}


	double mat[9];
	EulerToMatrix(-ang, mat);
	for(int i = 0; i < vn; ++i){
		Vec3 pos = polys.vertices[i];
		Vec3 nrm = polys.normals[i];

		Vec3 pos1 = ((pos-trans)/scale)*2.0*ext;
		Vec3 nrm1 = nrm;

		pos[0] = mat[0]*pos1[0]+mat[1]*pos1[1]+mat[2]*pos1[2];
		pos[1] = mat[3]*pos1[0]+mat[4]*pos1[1]+mat[5]*pos1[2];
		pos[2] = mat[6]*pos1[0]+mat[7]*pos1[1]+mat[8]*pos1[2];
		nrm[0] = mat[0]*nrm1[0]+mat[1]*nrm1[1]+mat[2]*nrm1[2];
		nrm[1] = mat[3]*nrm1[0]+mat[4]*nrm1[1]+mat[5]*nrm1[2];
		nrm[2] = mat[6]*nrm1[0]+mat[7]*nrm1[1]+mat[8]*nrm1[2];

		pos += cen;

		polys.vertices[i] = pos;
		polys.normals[i]  = nrm;
	}

	return true;
}


/*!
 * ���_�񂩂��AABB�̌���
 * @param[out] minp,maxp AABB�̍ő���W�C�ŏ����W
 * @param[in] vec_set ���_��
 * @param[in] start_index,end_index ���_��̌����͈�
 * @return �����ł�����true
 */
inline bool FindBBox(Vec3 &minp, Vec3 &maxp, 
					 const vector<Vec3> &vec_set, 
					 const int start_index, const int end_index)
{
	if((int)vec_set.size() == 0) return false;

	maxp = vec_set[start_index];
	minp = vec_set[start_index];

	for(int i = start_index+1; i < end_index; ++i){
		if(vec_set[i][0] > maxp[0]) maxp[0] = vec_set[i][0];
		if(vec_set[i][1] > maxp[1]) maxp[1] = vec_set[i][1];
		if(vec_set[i][2] > maxp[2]) maxp[2] = vec_set[i][2];
		if(vec_set[i][0] < minp[0]) minp[0] = vec_set[i][0];
		if(vec_set[i][1] < minp[1]) minp[1] = vec_set[i][1];
		if(vec_set[i][2] < minp[2]) minp[2] = vec_set[i][2];
	}

	return true;
}
/*!
 * ���_�񂩂��AABB�̌���
 * @param[out] minp,maxp AABB�̍ő���W�C�ŏ����W
 * @param[in] vec_set ���_��
 * @return �����ł�����true
 */
inline bool FindBBox(Vec3 &minp, Vec3 &maxp, const vector<Vec3> &vec_set)
{
	return FindBBox(minp, maxp, vec_set, 0, (int)vec_set.size());
}

/*!
 * ���_���AABB�ɍ����悤��Fit������
 * @param[in] ctr AABB���S���W
 * @param[in] sl  AABB�̕ӂ̒���(1/2)
 * @param[in] vec_set ���_��
 * @param[in] start_index,end_index ���_��̌����͈�
 */
inline bool FitVertices(const Vec3 &ctr, const Vec3 &sl, 
						vector<Vec3> &vec_set, 
						const int start_index, const int end_index)
{
	Vec3 ctr0, sl0, maxp, minp;

	// ���݂�BBox�̑傫���𒲂ׂ�
	FindBBox(minp, maxp, vec_set, start_index, end_index);
			
	sl0  = (maxp-minp)/2.0;
	ctr0 = (maxp+minp)/2.0;

	int max_axis = ( ( (sl0[0] > sl0[1]) && (sl0[0] > sl0[2]) ) ? 0 : ( (sl0[1] > sl0[2]) ? 1 : 2 ) );
	int min_axis = ( ( (sl0[0] < sl0[1]) && (sl0[0] < sl0[2]) ) ? 0 : ( (sl0[1] < sl0[2]) ? 1 : 2 ) );
	double size_conv = sl[max_axis]/sl0[max_axis];

	// �S�Ă̒��_��bbox�ɂ��킹�ĕϊ�
	for(int i = start_index; i < end_index; ++i){
		vec_set[i] = (vec_set[i]-ctr0)*size_conv+ctr;
	}

	return true;
}

/*!
 * ���_���AABB�ɍ����悤��Fit������
 * @param[in] ctr AABB���S���W
 * @param[in] sl  AABB�̕ӂ̒���(1/2)
 * @param[in] vec_set ���_��
 */
inline bool FitVertices(const Vec3 &ctr, const Vec3 &sl, vector<Vec3> &vec_set)
{
	FitVertices(ctr, sl, vec_set, 0, (int)vec_set.size());
	return true;
}

/*!
 * ���_�@���v�Z
 * @param[in] vrts ���_���W
 * @param[in] nvrts ���_��
 * @param[in] tris �O�p�`�|���S���􉽏��
 * @param[in] ntris �O�p�`�|���S����
 * @param[out] nrms �@��
 * @param[out] nnrms �@����(=���_��)
static void CalVertexNormalsFromVBO(GLuint vrts_vbo, GLuint tris_vbo, GLuint nrms_vbo, uint nvrts, uint ntris)
{
	vector<Vec3> nrms_temp;
	nrms_temp.resize(nvrts);

	glBindBuffer(GL_ARRAY_BUFFER, vrts_vbo);
	float *vrts = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_ONLY);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, tris_vbo);
	uint *tris = (uint*)glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_READ_ONLY);

	// Set all normals to 0.
	for(uint i = 0; i < nvrts; i++){
		nrms_temp[i][0] = 0;
		nrms_temp[i][1] = 0;
		nrms_temp[i][2] = 0;
	}

	// Calculate normals.
	for(uint i = 0; i < ntris; i++){
		Vec3 vec1, vec2, normal;
		uint id0, id1, id2;
		id0 = tris[3*i+0];
		id1 = tris[3*i+1];
		id2 = tris[3*i+2];

		for(int j = 0; j < 3; ++j){
			vec1[j] = vrts[3*id1+j]-vrts[3*id0+j];
			vec2[j] = vrts[3*id2+j]-vrts[3*id0+j];
		}
		normal = Unit(cross(vec2, vec1));

		nrms_temp[id0] += normal;
		nrms_temp[id1] += normal;
		nrms_temp[id2] += normal;
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, nrms_vbo);
	float *nrms = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);

	// Normalize normals.
	for(uint i = 0; i < nvrts; i++){
		normalize(nrms_temp[i]);

		for(int j = 0; j < 3; ++j){
			nrms[3*i+j] = nrms_temp[i][j];
		}
	}

	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
 */


/*!
 * �V�~�����[�V������Ԃ𕢂��O���b�h�̎Z�o
 *  - �ő�O���b�h��nmax���w�肵�āC�ő�ӂ����̃O���b�h���ɂȂ�悤�ɑ��̕ӂ𕪊�����D
 *  - �O���b�h�͗����̃Z���ō\�������D
 * @param[in] minp,maxp �V�~�����[�V������Ԃ͈̔�
 * @param[in] nmax �ő�O���b�h��
 * @param[out] h �����̃Z���̕�
 * @param[out] n �e���̃Z����
 * @param[in] extend ��Ԃ����S�ɕ������߂̊g����[0,1]
 * @return �����Ȃ��0
 */
inline int CalMeshDiv(Vec3 &minp, Vec3 maxp, int nmax, double &h, int n[3], double extend)
{
	Vec3 l = maxp-minp;
	minp -= 0.5*extend*l;
	l *= 1.0+extend;

	double max_l = 0;
	int max_axis = 0;
	for(int i = 0; i < 3; ++i){
		if(l[i] > max_l){
			max_l = l[i];
			max_axis = i;
		}
	}

	h = max_l/nmax;
	for(int i = 0; i < 3; ++i){
		n[i] = (int)(l[i]/h)+1;
	}

	return 0;
}


/*!
 * ���b�V���`��p��VBO���m��
 * @param[in] max_verts �ő咸�_��
 * @param[in] uVrtVBO ���_FBO
 * @param[in] uNrmVBO �@��FBO
 * @param[in] uTriVBO ���b�V��FBO
 */
inline bool AssignArrayBuffers(int max_verts, int dim, GLuint &uVrtVBO, GLuint &uNrmVBO, GLuint &uTriVBO)
{
	// ���_VBO
	if(!uVrtVBO) glGenBuffers(1, &uVrtVBO);
	glBindBuffer(GL_ARRAY_BUFFER, uVrtVBO);
	glBufferData(GL_ARRAY_BUFFER, max_verts*dim*sizeof(float), 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// �@��VBO
	if(!uNrmVBO) glGenBuffers(1, &uNrmVBO);
	glBindBuffer(GL_ARRAY_BUFFER, uNrmVBO);
	glBufferData(GL_ARRAY_BUFFER, max_verts*dim*sizeof(float), 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// ���b�V��VBO
	if(!uTriVBO) glGenBuffers(1, &uTriVBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, uTriVBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, max_verts*3*3*sizeof(unsigned int), 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	return true;
}


/*!
 * FBO�Ƀf�[�^��ݒ�
 * @param[in] uVrtVBO ���_FBO
 * @param[in] uNrmVBO �@��FBO
 * @param[in] uTriVBO ���b�V��FBO
 */
inline bool SetFBOFromArray(GLuint uVrtVBO, GLuint uNrmVBO, GLuint uTriVBO, 
							vector<Vec3> &vrts, vector<Vec3> &nrms, vector<rxFace> &face)
{
	int nv = (int)vrts.size();
	int nm = (int)face.size();
	int nn = nv;

	// ���_�A���C�Ɋi�[
	glBindBuffer(GL_ARRAY_BUFFER, uVrtVBO);
	float *vrt_ptr = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);

	for(int i = 0; i < nv; ++i){
		for(int j = 0; j < 3; ++j){
			vrt_ptr[3*i+j] = (float)vrts[i][j];
		}
	}

	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// �@�����̎擾
	glBindBuffer(GL_ARRAY_BUFFER, uNrmVBO);
	float *nrm_ptr = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);

	for(int i = 0; i < nn; ++i){
		for(int j = 0; j < 3; ++j){
			nrm_ptr[3*i+j] = (float)nrms[i][j];
		}
	}

	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// �ڑ����̎擾
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, uTriVBO);
	unsigned int *tri_ptr = (unsigned int*)glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY);

	for(int i = 0; i < nm; ++i){
		for(int j = 0; j < 3; ++j){
			tri_ptr[3*i+j] = face[i][j];
		}
	}

	glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	return true;
}

/*!
 * �z�X�g���z��Ƀf�[�^��ݒ�
 * @param[out] vrts ���_���W
 * @param[out] nrms ���_�@��
 * @param[out] tris ���b�V��
 */
inline bool SetArrayFromFBO(GLuint uVrtVBO, GLuint uNrmVBO, GLuint uTriVBO, 
							vector<Vec3> &vrts, vector<Vec3> &nrms, vector<rxFace> &face, int nvrts, int ntris)
{
	// ���_�A���C�Ɋi�[
	glBindBuffer(GL_ARRAY_BUFFER, uVrtVBO);
	float *vrt_ptr = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_ONLY);

	vrts.resize(nvrts);
	for(int i = 0; i < nvrts; ++i){
		vrts[i][0] = vrt_ptr[4*i];
		vrts[i][1] = vrt_ptr[4*i+1];
		vrts[i][2] = vrt_ptr[4*i+2];
	}

	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// �@�����̎擾
	glBindBuffer(GL_ARRAY_BUFFER, uNrmVBO);
	float *nrm_ptr = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_ONLY);

	nrms.resize(nvrts);
	for(int i = 0; i < nvrts; ++i){
		nrms[i][0] = nrm_ptr[4*i];
		nrms[i][1] = nrm_ptr[4*i+1];
		nrms[i][2] = nrm_ptr[4*i+2];
	}

	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// �ڑ����̎擾
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, uTriVBO);
	unsigned  *tri_ptr = (unsigned int*)glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_READ_ONLY);

	face.resize(ntris);
	for(int i = 0; i < ntris; ++i){
		face[i].vert_idx.resize(3);
		face[i][0] = tri_ptr[3*i];
		face[i][1] = tri_ptr[3*i+1];
		face[i][2] = tri_ptr[3*i+2];
	}

	glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	return true;
}




#endif // #ifndef _RX_MESH_H_
