/*!
  @file rx_trackball.h
	
  @brief �ȈՃg���b�N�{�[������
 
  @author Makoto Fujisawa
  @date 2008
*/
// FILE --rx_trackball.h--

#ifndef _RX_TRACKBALL_H_
#define _RX_TRACKBALL_H_


//-----------------------------------------------------------------------------
// �l�����֐�
//-----------------------------------------------------------------------------
void QuatLerp(double p[], const double q0[], const double q1[], const double t);
void QuatMultiLerp(double p[], const double t[], const double q[][4], const int n, const double u);
void SetQuat(double q[], const double ang, const double ax, const double ay, const double az);


//-----------------------------------------------------------------------------
// �g���b�N�{�[���֐�
//-----------------------------------------------------------------------------
class rxTrackball
{
public:
	rxTrackball();
	~rxTrackball();

	// �������Ɛݒ�
	void Init(void);			//!< �g���b�N�{�[�������̏�����
	void SetRegion(int w, int h);	//!< ��ʗ̈�w��
	void SetSpace(double l);		//!< 3D��Ԃ̑傫���w��

	// �}�E�X�h���b�O
	void Start(int x, int y, int button);	//!< �h���b�O�J�n
	void Motion(int x, int y, bool last = false);				//!< �h���b�O��
	void Stop(int x, int y);				//!< �h���b�O��~

	void TrackballLastRot(void);
	void TrackballRot(double vx, double vy);

	// �A�N�Z�X���\�b�h
	void GetQuaternion(double q[4]) const;
	double GetScaling(void) const;		//!< �X�P�[�����O�ʂ�Ԃ�
	void GetScaling(double &s) const;
	double GetTranslation(int i) const;
	void GetTranslation(double t[2]) const;

	double *GetRotation(void);		//!< ��]�̕ϊ��s���Ԃ�
	double *GetQuaternionR(void);	//!< �l������Ԃ�(q[4])
	double *GetTranslationR(void);	//!< ���s�ړ��ʂ�Ԃ�(t[2])
	double *GetScalingR(void);		//!< �X�P�[�����O�ʂ�Ԃ�(s)

	void GetTransform(double m[16]);

	void GetLastVeloc(int &vx, int &vy);

	void CalGlobalPos(double dst[4], const double src[4]);
	void CalLocalPos(double dst[4], const double src[4]);

	void CalGlobalRot(double dst[4], const double src[4]);
	void CalLocalRot(double dst[4], const double src[4]);

	void GetViewPosition(double pos[3]);
	void GetViewDirection(double dir[3]);

	// �`��ݒ�
	void ApplyRotation(void);		//!< ��]��OpenGL�ɐݒ�
	void ApplyScaling(void);		//!< �X�P�[�����O��OpenGL�ɐݒ�
	void ApplyTranslation(void);	//!< ���s�ړ��ʂ�OpenGL�ɐݒ�

	void Apply(void);

	void ApplyQuaternion(const double q[4]);		//!< ��]��OpenGL�ɐݒ�


	void SetScaling(double z);					
	void SetTranslation(double x, double y);
	void SetQuaternion(double q[4]);
	void SetRotation(double ang, double x, double y, double z);
	void SetRotation(double rot[16]);

	void AddRotation(double ang, double x, double y, double z);
	void GetLastRotation(double &ang, double &x, double &y, double &z);

	void GetRayTo(int x, int y, double fov, double ray_to[3]);

private:
	double m_fTransScale;		//!< �}�E�X�̑��Έʒu���X�P�[�����O�C���s�ړ��ʂ̊��Z�W��

	int m_iSx, m_iSy;	//!< �h���b�O�J�n�ʒu
	int m_iPx, m_iPy;	//!< �h���b�O�J�n�ʒu
	double m_fW, m_fH;	//!< �}�E�X�̐�Έʒu���E�B���h�E���ł̑��Έʒu�̊��Z�W��

	int m_iVeloc[2];

	double m_fQuatRot[4];		//!< ��](�N�H�[�^�j�I��)
	double m_fQuatIncRot[4];	//!< �h���b�O���̉�]������(�N�H�[�^�j�I��)
	double m_fMatRot[16];		//!< ��]�̕ϊ��s��
	
	double m_fTransDist[2];		//!< �J�����p��
	double m_fScaleDist;		//!< �X�P�[�����O
	int m_iDrag;				//!< �h���b�O�����ۂ�(1:���h���b�O, 2:�E�h���b�O, 3:�~�h���h���b�O)

	double m_fLastRot[4];		//!< �Ō�̉�]

};


#endif // #ifndef _RX_TRACKBALL_H_