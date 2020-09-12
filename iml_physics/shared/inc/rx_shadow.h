/*!	@file rx_shadow.h
	
	@brief �V���h�E�}�b�v�@�ɂ��e�t��
 
	@author Makoto Fujisawa
	@date   2009
*/

#ifndef _RX_SHADOW_MAP_H_
#define _RX_SHADOW_MAP_H_


//-----------------------------------------------------------------------------
// �C���N���[�h�t�@�C��
//-----------------------------------------------------------------------------
#include "rx_matrix.h"
#include "rx_utility.h"

#include "rx_shaders.h"

#include <GL/glew.h>
#include <GL/glut.h>

#pragma comment (lib, "glew32.lib")


//-----------------------------------------------------------------------------
// ��`�E�萔
//-----------------------------------------------------------------------------
using namespace std;

#define RX_MAX_SPLITS 4
#define RX_LIGHT_FOV 45.0

#define RX_FAR_DIST 20.0f


//-----------------------------------------------------------------------------
// HACK:������
//-----------------------------------------------------------------------------
struct rxFrustum
{
	double Near;
	double Far;
	double FOV;	// radian(OpenGL��degree�Ȃ̂ŕϊ����邱��)
	double Ratio;
	Vec3 Point[8];
};

//-----------------------------------------------------------------------------
// MARK:rxShadowMap
//-----------------------------------------------------------------------------
class rxShadowMap
{
	//
	// �V���h�E�}�b�v�ϐ�
	//
	int m_iFrustumCutNum;	//!< �����K�w�̃V���h�E�}�b�v��p����ꍇ�̊K�w��
	GLuint m_iFBDepth;		//!< �������猩���Ƃ��̃f�v�X���i�[����Framebuffer object
	GLuint m_iTexDepth;		//!< m_iFBDepth��attach����e�N�X�`��
	GLuint m_iDepthSize;	//!< �f�v�X���i�[����e�N�X�`���̃T�C�Y

	vector<rxFrustum> m_vFrustums;	//!< ������
	vector<rxMatrix4> m_vMatShadowCPM;

	int m_iShadowType;

	vector<rxGLSL> m_vShadGLSL;
	rxGLSL m_GLSLView;

	Vec3 m_v3EyePos, m_v3EyeDir;
	Vec3 m_v3LightPos;

	int m_iWidth, m_iHeight;

	void (*m_fpDraw)(void);

	double m_fFarDist;

public:
	//! �f�t�H���g�R���X�g���N�^
	rxShadowMap()
	{
		m_iShadowType = 0;
		m_fFarDist = RX_FAR_DIST;
	}

	//! �f�X�g���N�^
	~rxShadowMap(){}


	//
	// GLSL
	//
	// GLSL�v���O�����R���p�C���E�����N
	void SetShadowGLSL(vector<rxGLSL> &gss);

	GLuint GetGLSLProg(void){ return m_vShadGLSL[m_iShadowType].Prog; }


	//
	// �e�̐������@
	//
	void SetShadowType(int n);
	int  GetShadowTypeNum(void) const { return (int)m_vShadGLSL.size(); }
	string GetCurrentShadowType(void);


	//
	// ������
	//
	// ������̌v�Z
	inline rxFrustum CalFrustum(int w, int h, double fov);
	void CalAllFrustums(int w, int h, double fov);

	// Far�̐ݒ�
	void  SetFarDist(double f){ m_fFarDist = f; }
	float GetFarDist(void){ return m_fFarDist; }

	// ������̕������̑I��
	void SetFrustumCutNum(int n);
	int  GetFrustumCutNum(void) const;
	void IncreaseFrustumCutNum(int d = 1);

	// �������8�R�[�i�[���_���v�Z����
	void UpdateFrustumPoints(rxFrustum &f, const Vec3 &center, const Vec3 &view_dir);

	// �J�������_��Ԃł̊e������X���C�X��near��far���v�Z����
	void UpdateSplitDist(vector<rxFrustum> &fs, float nd, float fd);

	// ��������̕��s���e�s��̌v�Z
	float ApplyCropMatrix(rxFrustum &f, const vector<Vec3> &bcns, const vector<Vec3> &bsls);

	void CameraInverse(float dst[16], float src[16]);


	//
	// �V���h�E�}�b�v
	//
	// �V���h�E�}�b�v�̏�����
	void InitShadowMap(int w, int h, double fov, int cut = 4, int size = 2048);

	// �V���h�E�}�b�v(�f�v�X�e�N�X�`��)�̍쐬
	void MakeShadowMap(const Vec4 &light_dir, const Vec3 &eye_pos, const Vec3 &eye_dir, 
					   void (*fpDraw)(void), 
					   const vector<Vec3> &bcns, const vector<Vec3> &bsls);

	// �e�t���ŃV�[���`��
	void RenderSceneWithShadow(void (*fpDraw)(void), int w, int h, int tex = 1);


	//
	// �V���h�E�}�b�v�m�F�`��
	//
	// �J��������p�̕`��
	void OverviewCam(void (*fpDraw)(void), int w, int h);

	// �f�v�X�}�b�v�̕`��
	void DrawDepthTex(void);

};


/*!
 * �V���h�E�}�b�v�̏�����
 * @param[in] w,h  �E�B���h�E�T�C�Y
 * @param[in] fov  ����p
 * @param[in] cut ������̕�����
 * @param[in] size �V���h�E�}�b�v�̉𑜓x
 */
inline void rxShadowMap::InitShadowMap(int w, int h, double fov, int cut, int size)
{
	// MARK:InitShadowMap
	m_iFrustumCutNum = cut;
	m_iDepthSize = size;

	glGenFramebuffersEXT(1, &m_iFBDepth);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_iFBDepth);
	glDrawBuffer(GL_NONE);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	glGenTextures(1, &m_iTexDepth);
	cout << "tex depth : " << m_iTexDepth << endl;

	glBindTexture(GL_TEXTURE_2D_ARRAY_EXT, m_iTexDepth);
	glTexImage3D(GL_TEXTURE_2D_ARRAY_EXT, 0, GL_DEPTH_COMPONENT24, m_iDepthSize, m_iDepthSize, RX_MAX_SPLITS, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D_ARRAY_EXT, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D_ARRAY_EXT, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D_ARRAY_EXT, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D_ARRAY_EXT, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D_ARRAY_EXT, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);

	SetShadowGLSL(m_vShadGLSL);
	//m_GLSLView = CreateGLSLFromFile("shader/shadow_view.vs", "shader/shadow_view.fs", "view");
	m_GLSLView = CreateGLSL(shadow_view_vs, shadow_view_fs, "view");

	m_vFrustums.resize(RX_MAX_SPLITS);
	for(int i = 0; i < (int)m_vFrustums.size(); ++i){
		m_vFrustums[i] = CalFrustum(w, h, fov);
	}

	m_vMatShadowCPM.resize(RX_MAX_SPLITS);
	for(int i = 0; i < (int)m_vMatShadowCPM.size(); ++i){
		m_vMatShadowCPM[i].MakeIdentity();
	}
}


/*!
 * �V���h�E�}�b�v(�f�v�X�e�N�X�`��)�̍쐬
 * @param[in] light_pos �����ʒu(�_����,�w��������)
 * @param[in] eye_pos   ���_�ʒu
 * @param[in] eye_dir   ���_����
 * @param[in] fpDraw	�`��֐��̃|�C���^
 * @param[in] bcns      �`��I�u�W�F�N�gBBox�̒��S���W
 * @param[in] bsls      �`��I�u�W�F�N�gBBox�̕ӂ̒����̔���
 */
inline void rxShadowMap::MakeShadowMap(const Vec4 &light_dir, const Vec3 &eye_pos, const Vec3 &eye_dir, 
									   void (*fpDraw)(void), 
									   const vector<Vec3> &bcns, const vector<Vec3> &bsls)
{
	// MARK:MakeShadowMap
	if(!m_iFrustumCutNum){ return; }

	float shad_modelview[16];
	//glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	// glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// �����ʒu�����_�ɐݒ�
	//gluLookAt(light_pos[0], light_pos[1], light_pos[2], 0, 0, 0, -1.0f, 0.0f, 0.0f);
	gluLookAt(0, 0, 0, -light_dir[0], -light_dir[1], -light_dir[2], -1.0f, 0.0f, 0.0f);
	glGetFloatv(GL_MODELVIEW_MATRIX, shad_modelview);

	// �f�v�X�e�N�X�`���pFBO
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_iFBDepth);
	glPushAttrib(GL_VIEWPORT_BIT);					// �X�N���[���r���[�|�[�g���ꎞ�Ҕ�
	glViewport(0, 0, m_iDepthSize, m_iDepthSize);	// �V���h�E�}�b�v�̃T�C�Y�ɍ��킹��

	// Z�t�@�C�e�B���O��h�����߂Ƀ|���S���I�t�Z�b�g��ݒ�
	//glPolygonOffset(0.75, 1.0);
	glPolygonOffset(1.0, 4096.0);
	glEnable(GL_POLYGON_OFFSET_FILL);

	glDisable(GL_CULL_FACE);

	// ������̕����̍X�V
	UpdateSplitDist(m_vFrustums, 0.01f, (float)m_fFarDist);

	//Vec3 eye_pos = g_viewObject.CalInverseTransform(Vec3(0.0));
	//Vec3 eye_dir = Unit(g_viewObject.CalInverseRotation(Vec3(0.0, 0.0, -1.0)));

	//eye_dir *= -1.0;

	//g_cbDraw << "eye_pos = " << eye_pos << "\n";
	//g_cbDraw << "eye_dir = " << eye_dir << "\n";
	m_v3EyePos = eye_pos;
	m_v3EyeDir = eye_dir;

	m_v3LightPos = Vec3(light_dir[0], light_dir[1], light_dir[2]);

	for(int i = 0; i < m_iFrustumCutNum; ++i){
		// ���[���h���W�ł̎�����X���C�X�̋��E�_���X�V
		UpdateFrustumPoints(m_vFrustums[i], eye_pos, eye_dir);

		// ��������̕��s���e�s���������X���C�X�̋��E�_�����Ƃɐݒ�
		float minZ = ApplyCropMatrix(m_vFrustums[i], bcns, bsls);

		// 3D�e�N�X�`����p�����f�v�X�}�b�v
		glFramebufferTextureLayerEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, m_iTexDepth, 0, i);

		glClear(GL_DEPTH_BUFFER_BIT);

		// �������猩���V�[���̕`��
		glMatrixMode(GL_MODELVIEW);
		fpDraw();
		
		glMatrixMode(GL_PROJECTION);
		glMultMatrixf(shad_modelview);

		float shadow_cpm[16];
		glGetFloatv(GL_PROJECTION_MATRIX, shadow_cpm);
		m_vMatShadowCPM[i].SetValueT<float>(shadow_cpm);

		//glGetFloatv(GL_PROJECTION_MATRIX, shad_cpm[i]);
	}


	glDisable(GL_POLYGON_OFFSET_FILL);
	glPopAttrib(); 
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	glEnable(GL_TEXTURE_2D);
	glEnable(GL_CULL_FACE);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

/*!
 * �e�t���ŃV�[���`��
 * @param[in] fpDraw �`��֐��̃|�C���^
 * @param[in] w,h	�E�B���h�E�T�C�Y
 */
inline void rxShadowMap::RenderSceneWithShadow(void (*fpDraw)(void), int w, int h, int tex)
{
	// MARK:RenderSceneWithShadow
	float cam_proj[16];
	float cam_modelview[16];
	float cam_inverse_modelview[16];
	float far_bound[RX_MAX_SPLITS];
	const float bias[16] = {	0.5f, 0.0f, 0.0f, 0.0f, 
								0.0f, 0.5f, 0.0f, 0.0f,
								0.0f, 0.0f, 0.5f, 0.0f,
								0.5f, 0.5f, 0.5f, 1.0f	};


	// �e�v�Z�̂��߂Ƀ��f���r���[�s��̋t�s����v�Z
	glGetFloatv(GL_MODELVIEW_MATRIX, cam_modelview);
	CameraInverse(cam_inverse_modelview, cam_modelview);

	//g_cbDraw << m_iTexDepth << "\n";

	if(!m_iFrustumCutNum){
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(45.0, (double)w/(double)h, m_vFrustums[0].Near, m_vFrustums[0].Far);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();

		fpDraw();

		glPopMatrix();
	}
	else{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(45.0, (double)w/(double)h, m_vFrustums[0].Near, m_vFrustums[m_iFrustumCutNum-1].Far);
		glGetFloatv(GL_PROJECTION_MATRIX, cam_proj);

		//
		// �f�v�X�}�b�v�̃o�C���h
		//
		glBindTexture(GL_TEXTURE_2D_ARRAY_EXT, m_iTexDepth);
		if(m_iShadowType >= 4){
			glTexParameteri(GL_TEXTURE_2D_ARRAY_EXT, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
		}
		else{
			glTexParameteri(GL_TEXTURE_2D_ARRAY_EXT, GL_TEXTURE_COMPARE_MODE, GL_NONE);
		}

		for(int i = m_iFrustumCutNum; i < RX_MAX_SPLITS; ++i){
			far_bound[i] = 0;
		}

		// �X���C�X���Ƃ̃e�N�X�`�����A�N�e�B�u��
		for(int i = 0; i < m_iFrustumCutNum; ++i){
			float light_m[16];

			// �J�����������W�n�ł�Far���v�Z(cam_proj*(0,0,Far,1)^t���v�Z��,[0,1]�ɐ��K��)
			far_bound[i] = (float)(0.5f*(-m_vFrustums[i].Far*cam_proj[10]+cam_proj[14])/m_vFrustums[i].Far+0.5f);

			// �ۑ����Ă������������W�n�ւ̕ϊ��s���p���āC
			// �������W�n�ɕϊ����ăe�N�X�`����\��t��
			glActiveTexture(GL_TEXTURE0+(GLenum)i);
			glMatrixMode(GL_TEXTURE);
			glLoadMatrixf(bias);

			float shadow_cpm[16];
			m_vMatShadowCPM[i].GetValueT<float>(shadow_cpm);

			glMultMatrixf(shadow_cpm);
			glMultMatrixf(cam_inverse_modelview);
			
			// compute a normal matrix for the same thing (to transform the normals)
			// Basically, N = ((L)^-1)^-t
			glGetFloatv(GL_TEXTURE_MATRIX, light_m);
			rxMatrix4 nm;
			nm.SetValueT<float>(light_m);
			nm = nm.Inverse();
			nm = nm.Transpose();

			float m[16];
			nm.GetValueT<float>(m);
			glActiveTexture(GL_TEXTURE0 + (GLenum)(i+4));
			glMatrixMode(GL_TEXTURE);
			glLoadMatrixf(m);
		}

		// GLSL
		glUseProgram(m_vShadGLSL[m_iShadowType].Prog);
		glUniform1i(glGetUniformLocation(m_vShadGLSL[m_iShadowType].Prog, "stex"), 0);	// depth-maps
		glUniform1i(glGetUniformLocation(m_vShadGLSL[m_iShadowType].Prog, "tex"), 1);	// other tex
		glUniform4fv(glGetUniformLocation(m_vShadGLSL[m_iShadowType].Prog, "far_d"), 1, far_bound);
		if(m_iShadowType >= 4){
			glUniform2f(glGetUniformLocation(m_vShadGLSL[m_iShadowType].Prog, "texSize"), (float)m_iDepthSize, 1.0f/(float)m_iDepthSize);
		}

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();

		fpDraw();

		glPopMatrix();

		glUseProgram(0);
	}
}

/*!
 * �e�t���ŃV�[���`��
 * @param[in] fpDraw �`��֐��̃|�C���^
 * @param[in] w,h	�E�B���h�E�T�C�Y
 */
inline void rxShadowMap::OverviewCam(void (*fpDraw)(void), int w, int h)
{
	// HACK:OverviewCam
	glPushAttrib(GL_VIEWPORT_BIT | GL_DEPTH_BUFFER_BIT);

	glViewport(m_iWidth-129, 0, 128, 128);
	glEnable(GL_DEPTH_TEST);
	glClear(GL_DEPTH_BUFFER_BIT);


	glActiveTexture(GL_TEXTURE0);
	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_LIGHTING);
	glPointSize(10);
	glColor3f(1.0f, 1.0f, 0.0f);
	gluLookAt(0, m_fFarDist/2, 0, 0, 0, 0, 0, 0, 1.0f);

	glScalef(0.2f, 0.2f, 0.2f);
	glRotatef(20, 1, 0, 0);
	for(int i = 0; i < m_iFrustumCutNum; ++i){
		glBegin(GL_LINE_LOOP);
		for(int j=0; j<4; j++)
			glVertex3dv(m_vFrustums[i].Point[j].data);

		glEnd();
		glBegin(GL_LINE_LOOP);
		for(int j=4; j<8; j++)
			glVertex3dv(m_vFrustums[i].Point[j].data);
		glEnd();
	}

	for(int j = 0; j < 4; ++j){
		glBegin(GL_LINE_STRIP);
		glVertex3dv(m_v3EyePos.data);
		for(int i = 0; i < m_iFrustumCutNum; ++i){
			glVertex3dv(m_vFrustums[i].Point[j].data);
		}

		glVertex3dv(m_vFrustums[m_iFrustumCutNum-1].Point[j+4].data);
		glEnd();
	}

	glPushAttrib(GL_LIGHTING_BIT);

	GLfloat light_pos[4];
	for(int i = 0; i < 3; ++i) light_pos[i] = (GLfloat)m_v3LightPos[i];
	light_pos[3] = 1.0;
	glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
	glColor3f(0.9f, 0.9f, 1.0f);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	fpDraw();
	
	glPopAttrib();
	glPopAttrib();
}

/*!
 * �f�v�X�}�b�v�̕`��
 */
inline void rxShadowMap::DrawDepthTex(void)
{
	int loc;
	glPushAttrib(GL_VIEWPORT_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glUseProgram(m_GLSLView.Prog);
	glUniform1i(glGetUniformLocation(m_GLSLView.Prog, "tex"), 0);
	loc = glGetUniformLocation(m_GLSLView.Prog, "layer");
	for(int i = 0; i < m_iFrustumCutNum; ++i){
		glViewport(130*i, 0, 128, 128);
		glBindTexture(GL_TEXTURE_2D_ARRAY_EXT, m_iTexDepth);
		glTexParameteri( GL_TEXTURE_2D_ARRAY_EXT, GL_TEXTURE_COMPARE_MODE, GL_NONE);
		glUniform1f(loc, (float)i);
		glBegin(GL_QUADS);
		glVertex3f(-1.0f, -1.0f, 0.0f);
		glVertex3f( 1.0f, -1.0f, 0.0f);
		glVertex3f( 1.0f,  1.0f, 0.0f);
		glVertex3f(-1.0f,  1.0f, 0.0f);
		glEnd();

	}
	glUseProgram(0);

	glEnable(GL_CULL_FACE);
	glPopAttrib();
}


/*!
 * �e�̐������@��I��
 * @param[in] n �������@�ԍ�
 */
inline void rxShadowMap::SetShadowType(int n)
{
	int size = (int)m_vShadGLSL.size();
	if(n >= size) n = size-1;
	if(n < 0) n = 0;

	m_iShadowType = n;
}

/*!
 * �ݒ肳��Ă���e�̐������@���擾
 * @return �������@(������)
 */
inline string rxShadowMap::GetCurrentShadowType(void)
{
	return m_vShadGLSL[m_iShadowType].Name;
}



/*!
 * �V���h�E�}�b�v�pGLSL�v���O�����̃Z�b�g
 * @param[out] gss GLSL�v���O����
 */
inline void rxShadowMap::SetShadowGLSL(vector<rxGLSL> &gss)
{
	// MARK:SetShadowGLSL
	//string vert = "shader/shadow.vs";

	gss.clear();
	gss.push_back(CreateGLSL(shadow_vs, shadow_single_fs, 			"Normal Mode"));
	gss.push_back(CreateGLSL(shadow_vs, shadow_single_hl_fs, 		"Show Splits"));
	gss.push_back(CreateGLSL(shadow_vs, shadow_multi_fs, 			"Smooth shadows"));
	gss.push_back(CreateGLSL(shadow_vs, shadow_multi_noleak_fs,		"Smooth shadows, no leak"));
	gss.push_back(CreateGLSL(shadow_vs, shadow_pcf_fs,				"PCF"));
	gss.push_back(CreateGLSL(shadow_vs, shadow_pcf_trilinear_fs,	"PCF w/ trilinear"));
	gss.push_back(CreateGLSL(shadow_vs, shadow_pcf_4tap_fs,			"PCF w/ 4 taps"));
	gss.push_back(CreateGLSL(shadow_vs, shadow_pcf_8tap_fs,			"PCF w/ 8 random taps"));
	gss.push_back(CreateGLSL(shadow_vs, shadow_pcf_gaussian_fs,		"PCF w/ gaussian blur"));

	//gss.push_back(CreateGLSLFromFile(vert.c_str(), "shader/shadow_single.fs", 			"Normal Mode"));
	//gss.push_back(CreateGLSLFromFile(vert.c_str(), "shader/shadow_single_hl.fs", 		"Show Splits"));
	//gss.push_back(CreateGLSLFromFile(vert.c_str(), "shader/shadow_multi.fs", 			"Smooth shadows"));
	//gss.push_back(CreateGLSLFromFile(vert.c_str(), "shader/shadow_multi_noleak.fs",		"Smooth shadows, no leak"));
	//gss.push_back(CreateGLSLFromFile(vert.c_str(), "shader/shadow_pcf.fs",				"PCF"));
	//gss.push_back(CreateGLSLFromFile(vert.c_str(), "shader/shadow_pcf_trilinear.fs",	"PCF w/ trilinear"));
	//gss.push_back(CreateGLSLFromFile(vert.c_str(), "shader/shadow_pcf_4tap.fs",			"PCF w/ 4 taps"));
	//gss.push_back(CreateGLSLFromFile(vert.c_str(), "shader/shadow_pcf_8tap.fs",			"PCF w/ 8 random taps"));
	//gss.push_back(CreateGLSLFromFile(vert.c_str(), "shader/shadow_pcf_gaussian.fs",		"PCF w/ gaussian blur"));
}

/*!
 * �J�����s��̋t�ϊ��v�Z
 * @param[out] dst �t�ϊ��s��
 * @param[in] src �J�����s��
 * @return 
 */
inline void rxShadowMap::CameraInverse(float dst[16], float src[16])
{
	dst[0] = src[0];
	dst[1] = src[4];
	dst[2] = src[8];
	dst[3] = 0.0f;
	dst[4] = src[1];
	dst[5] = src[5];
	dst[6]  = src[9];
	dst[7] = 0.0f;
	dst[8] = src[2];
	dst[9] = src[6];
	dst[10] = src[10];
	dst[11] = 0.0f;
	dst[12] = -(src[12] * src[0]) - (src[13] * src[1]) - (src[14] * src[2]);
	dst[13] = -(src[12] * src[4]) - (src[13] * src[5]) - (src[14] * src[6]);
	dst[14] = -(src[12] * src[8]) - (src[13] * src[9]) - (src[14] * src[10]);
	dst[15] = 1.0f;
}


/*!
 * ������̌v�Z
 * @param[in] w,h �E�B���h�E�T�C�Y
 * @param[in] fov ����p(deg)
 * @return ������I�u�W�F�N�g
 */
inline rxFrustum rxShadowMap::CalFrustum(int w, int h, double fov)
{
	rxFrustum fr;
	fr.FOV = RX_TO_RADIANS(fov)+0.2;
	fr.Ratio = (double)w/(double)h;
	return fr;
}

/*!
 * �S������̌v�Z
 * @param[in] w,h �E�B���h�E�T�C�Y
 * @param[in] fov ����p(deg)
 */
inline void rxShadowMap::CalAllFrustums(int w, int h, double fov)
{
	m_iWidth = w;
	m_iHeight = h;
	for(int i = 0; i < (int)m_vFrustums.size(); ++i){
		m_vFrustums[i] = CalFrustum(w, h, fov);
	}
}

/*!
 * ������̕�������ݒ�
 * @param[in] n ������
 */
inline void rxShadowMap::SetFrustumCutNum(int n)
{
	if(n > 4) n = 4;
	if(n < 0) n = 0;

	m_iFrustumCutNum = n;
}
/*!
 * ������̕��������擾
 * @return ������
 */
inline int rxShadowMap::GetFrustumCutNum(void) const { return m_iFrustumCutNum; }


/*!
 * ������̕������𑝌�
 * @param[in] d ������(���̒l�Ō�����)
 */
inline void rxShadowMap::IncreaseFrustumCutNum(int d)
{
	m_iFrustumCutNum += d;

	if(m_iFrustumCutNum > 4) m_iFrustumCutNum = 4;
	if(m_iFrustumCutNum < 0) m_iFrustumCutNum = 0;
}

/*!
 * �������8�R�[�i�[���_���v�Z����
 * @param[in] f ������
 * @param[in] center ���_
 * @param[in] view_dir ��������
 */
inline void rxShadowMap::UpdateFrustumPoints(rxFrustum &f, const Vec3 &center, const Vec3 &view_dir)
{
	// MARK:UpdateFrustumPoints
	Vec3 up(0.0f, 1.0f, 0.0f);
	Vec3 right = cross(view_dir, up);

	Vec3 fc = center+view_dir*f.Far;
	Vec3 nc = center+view_dir*f.Near;

	right = Unit(right);
	up = Unit(cross(right, view_dir));

	// these heights and widths are half the heights and widths of
	// the near and far plane rectangles
	double near_height = tan(f.FOV/2.0f)*f.Near;
	double near_width  = near_height*f.Ratio;
	double far_height  = tan(f.FOV/2.0f)*f.Far;
	double far_width   = far_height*f.Ratio;

	f.Point[0] = nc-up*near_height-right*near_width;
	f.Point[1] = nc+up*near_height-right*near_width;
	f.Point[2] = nc+up*near_height+right*near_width;
	f.Point[3] = nc-up*near_height+right*near_width;

	f.Point[4] = fc-up*far_height-right*far_width;
	f.Point[5] = fc+up*far_height-right*far_width;
	f.Point[6] = fc+up*far_height+right*far_width;
	f.Point[7] = fc-up*far_height+right*far_width;
}

/*!
 * �J�������_��Ԃł̊e������X���C�X��near��far���v�Z����
 * @param[in] fs ������X���C�X
 * @param[in] nd �S�̂�near
 * @param[in] fd �S�̂�far
 */
inline void rxShadowMap::UpdateSplitDist(vector<rxFrustum> &fs, float nd, float fd)
{
	double lambda = 0.75;
	double ratio = fd/nd;
	fs[0].Near = nd;

	for(int i = 1; i < m_iFrustumCutNum; i++){
		double si = i/(double)m_iFrustumCutNum;

		fs[i].Near = lambda*(nd*powf(ratio, si))+(1-lambda)*(nd+(fd-nd)*si);
		fs[i-1].Far = fs[i].Near*1.05f;
	}

	if(m_iFrustumCutNum > 0){
		fs[m_iFrustumCutNum-1].Far = fd;
	}
}

/*!
 * ��������̕��s���e�s��̌v�Z
 *  - �ŏ��ɓK�؂�z�͈̔͂��v�Z���C���s���e���Z�b�g����D
 *  - ���Ɏ�����X���C�X�Ƀt�B�b�g����悤�ɕ��s�ړ�/�X�P�[�����O����
 * @param[in] f �Ή����鎋����X���C�X
 * @param[in] bcns �`��I�u�W�F�N�gBBox�̒��S���W
 * @param[in] bsls �`��I�u�W�F�N�gBBox�̕ӂ̒����̔���
 * @return z�̍ŏ��l
 */
inline float rxShadowMap::ApplyCropMatrix(rxFrustum &f, const vector<Vec3> &bcns, const vector<Vec3> &bsls)
{
	// MARK:ApplyCropMatrix
	float shad_modelview[16];
	float shad_proj[16];
	float shad_crop[16];
	float shad_mvp[16];
	float maxX = -1000.0f;
	float maxY = -1000.0f;
	float maxZ;
	float minX =  1000.0f;
	float minY =  1000.0f;
	float minZ;

	rxMatrix4 mvp;
	Vec4 transf;	
	
	//
	// ���݂̎�����(���������_�Ƃ���)��z�͈̔͂�����
	//
	// �������_�̃��f���r���[�s������o��
	glGetFloatv(GL_MODELVIEW_MATRIX, shad_modelview);
	mvp.SetValueT<float>(shad_modelview);

	// ������X���C�X��8���_�Ƀ��f���r���[�s����|���C�ő�E�ŏ���z�l��T��
	// (z�v�f�̂ݕK�v�Ȃ̂ňȉ��̂悤�ɒP�������\)
	 transf[2] = shad_modelview[2]*f.Point[0][0]+shad_modelview[6]*f.Point[0][1]+shad_modelview[10]*f.Point[0][1]+shad_modelview[14];
	mvp.multMatrixVec(Vec4(f.Point[0], 1.0f), transf);
	minZ = transf[2];
	maxZ = transf[2];
	for(int i = 1; i < 8; ++i){
		mvp.multMatrixVec(Vec4(f.Point[i], 1.0f), transf);

		if(transf[2] > maxZ) maxZ = transf[2];
		if(transf[2] < minZ) minZ = transf[2];
	}

	// �e���������I�u�W�F�N�g���m���ɔ͈͓��ɂ���悤��z�l��T���E�ύX
	for(int i = 0; i < (int)bcns.size(); ++i){
		Vec3 bcn = bcns[i];
		Vec3 bsl = bsls[i];
		mvp.multMatrixVec(Vec4(bcn[0], bcn[1], bcn[2], 1.0), transf);

		float diag = norm(bsl)/2.0f;
		if(transf[2]+diag > maxZ) maxZ = transf[2]+diag;
		//if(transf[2]-diag < minZ) minZ = transf[2]-diag;
	}


	//
	// �T������z�͈̔͂��瓊�e��ݒ�
	//
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//gluPerspective(RX_LIGHT_FOV, 1.0, minZ, maxZ); // �_�����̏ꍇ
	glOrtho(-1.0, 1.0, -1.0, 1.0, -maxZ, -minZ);

	glGetFloatv(GL_PROJECTION_MATRIX, shad_proj);
	glPushMatrix();

	glMultMatrixf(shad_modelview);
	glGetFloatv(GL_PROJECTION_MATRIX, shad_mvp);
	glPopMatrix();


	//
	// x,y�����̌���������̑傫���𒲐�
	//
	mvp.SetValueT<float>(shad_mvp);
	for(int i = 0; i < 8; ++i){
		mvp.multMatrixVec(Vec4(f.Point[i], 1.0f), transf);

		transf[0] /= transf[3];
		transf[1] /= transf[3];

		if(transf[0] > maxX) maxX = transf[0];
		if(transf[0] < minX) minX = transf[0];
		if(transf[1] > maxY) maxY = transf[1];
		if(transf[1] < minY) minY = transf[1];
	}

	float scaleX = 2.0f/(maxX-minX);
	float scaleY = 2.0f/(maxY-minY);
	float offsetX = -0.5f*(maxX+minX)*scaleX;
	float offsetY = -0.5f*(maxY+minY)*scaleY;

	// ������������g���~���O����s���glOrtho���瓾�����e�s��ɓK�p
	mvp.MakeIdentity();
	mvp(0, 0) = scaleX;
	mvp(1, 1) = scaleY;
	mvp(0, 3) = offsetX;
	mvp(1, 3) = offsetY;
	//mvp = mvp.Transpose();
	mvp.GetValueT<float>(shad_crop);

	glLoadMatrixf(shad_crop);
	glMultMatrixf(shad_proj);

	return minZ;
}



#endif // #ifndef _RX_SHADOW_MAP_H_