/*!
  @file rx_trackball.cpp

  @brief �ȈՃg���b�N�{�[������

  @author Makoto Fujisawa
  @date 2008
 */
// FILE --rx_trackball.cpp--

//-----------------------------------------------------------------------------
// �C���N���[�h�t�@�C��
//-----------------------------------------------------------------------------
#include <stdio.h>
#include <math.h>
#include "rx_trackball.h"

#include "GL/glut.h"


//-----------------------------------------------------------------------------
// ��`
//-----------------------------------------------------------------------------
template<class T> 
inline bool RX_FEQ2(const T &a, const T &b, const T &eps){ return (fabs(a-b) < eps); }

const double TB_PI = 3.14159265358979323846;
const double TB_ROT_SCALE = 2.0*TB_PI;	//!< �}�E�X�̑��Έʒu����]�p�̊��Z�W��

const double RX_FEQ_EPS = 1.0e-10;

// deg->rad �̕ϊ�(pi/180.0)
const double RX_DEGREES_TO_RADIANS = 0.0174532925199432957692369076848;
// rad->deg �̕ϊ�(180.0/pi)
const double RX_RADIANS_TO_DEGREES = 57.295779513082320876798154814114;

template<class T> 
inline T RX_TO_RADIANS(const T &x){ return static_cast<T>((x)*RX_DEGREES_TO_RADIANS); }
template<class T> 
inline T RX_TO_DEGREES(const T &x){ return static_cast<T>((x)*RX_RADIANS_TO_DEGREES); }

inline int IDX(int row, int col){ return (row | (col<<2)); }
//inline int IDX(int row, int col){ return (4*row+col); }


//-----------------------------------------------------------------------------
// MARK:�O���[�o���ϐ�
//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------
// MARK:�s��֐�
//-----------------------------------------------------------------------------
/*!
 * �s��ƃx�N�g���̂����Z(d = m x v)
 * @param[out] d ���ʂ̃x�N�g��
 * @param[in] m 4x4�s��
 * @param[in] v 4�����x�N�g��
 */
static void MulMatVec(double d[4], const double m[16], const double v[4])
{
	d[0] = (v[0]*m[IDX(0,0)]+v[1]*m[IDX(0,1)]+v[2]*m[IDX(0,2)]+v[3]*m[IDX(0,3)]);
	d[1] = (v[0]*m[IDX(1,0)]+v[1]*m[IDX(1,1)]+v[2]*m[IDX(1,2)]+v[3]*m[IDX(1,3)]);
	d[2] = (v[0]*m[IDX(2,0)]+v[1]*m[IDX(2,1)]+v[2]*m[IDX(2,2)]+v[3]*m[IDX(2,3)]);
	d[3] = (v[0]*m[IDX(3,0)]+v[1]*m[IDX(3,1)]+v[2]*m[IDX(3,2)]+v[3]*m[IDX(3,3)]);
}

/*!
 * �x�N�g���ƍs��̂����Z(d = v x m)
 * @param[out] d ���ʂ̃x�N�g��
 * @param[in] v 4�����x�N�g��
 * @param[in] m 4x4�s��
 */
static void MulVecMat(double d[4], const double v[4], const double m[16])
{
	d[0] = (v[0]*m[IDX(0,0)]+v[1]*m[IDX(1,0)]+v[2]*m[IDX(2,0)]+v[3]*m[IDX(3,0)]);
	d[1] = (v[0]*m[IDX(0,1)]+v[1]*m[IDX(1,1)]+v[2]*m[IDX(2,1)]+v[3]*m[IDX(3,1)]);
	d[2] = (v[0]*m[IDX(0,2)]+v[1]*m[IDX(1,2)]+v[2]*m[IDX(2,2)]+v[3]*m[IDX(3,2)]);
	d[3] = (v[0]*m[IDX(0,3)]+v[1]*m[IDX(1,3)]+v[2]*m[IDX(2,3)]+v[3]*m[IDX(3,3)]);
}

/*!
 * �P�ʍs�񐶐�
 * @param[out] m �P�ʍs��
 */
static void Identity(double m[16])
{
	for(int i = 0; i < 4; ++i){
		for(int j = 0; j < 4; ++j){
			if(i == j){
				m[IDX(i,j)] = 1.0;
			}
			else{
				m[IDX(i,j)] = 0.0;
			}
		}
	}
}

/*!
 * �s��R�s�[
 * @param[out] src �R�s�[���s��
 * @param[out] dst �R�s�[��s��
 */
static void CopyMat(const double src[16], double dst[16])
{
	for(int i = 0; i < 4; ++i){
		for(int j = 0; j < 4; ++j){
			dst[IDX(i,j)] = src[IDX(i,j)];
		}
	}
}

/*!
 * �t�s��(4x4)�v�Z
 * @param[out] inv_m �t�s��
 * @param[in] m ���̍s��
 */
static void Inverse(double inv_m[16], const double m[16])
{
	Identity(inv_m);

	double r1[8], r2[8], r3[8], r4[8];
	double *s[4], *tmprow;
	
	s[0] = &r1[0];
	s[1] = &r2[0];
	s[2] = &r3[0];
	s[3] = &r4[0];
	
	register int i,j,p,jj;
	for(i = 0; i < 4; ++i){
		for(j = 0; j < 4; ++j){
			s[i][j] = m[IDX(i, j)];

			if(i==j){
				s[i][j+4] = 1.0;
			}
			else{
				s[i][j+4] = 0.0;
			}
		}
	}

	double scp[4];
	for(i = 0; i < 4; ++i){
		scp[i] = double(fabs(s[i][0]));
		for(j=1; j < 4; ++j)
			if(double(fabs(s[i][j])) > scp[i]) scp[i] = double(fabs(s[i][j]));
			if(scp[i] == 0.0) return; // singular matrix!
	}
	
	int pivot_to;
	double scp_max;
	for(i = 0; i < 4; ++i){
		// select pivot row
		pivot_to = i;
		scp_max = double(fabs(s[i][i]/scp[i]));
		// find out which row should be on top
		for(p = i+1; p < 4; ++p)
			if(double(fabs(s[p][i]/scp[p])) > scp_max){
				scp_max = double(fabs(s[p][i]/scp[p])); pivot_to = p;
			}
			// Pivot if necessary
			if(pivot_to != i)
			{
				tmprow = s[i];
				s[i] = s[pivot_to];
				s[pivot_to] = tmprow;
				double tmpscp;
				tmpscp = scp[i];
				scp[i] = scp[pivot_to];
				scp[pivot_to] = tmpscp;
			}
			
			double mji;
			// perform gaussian elimination
			for(j = i+1; j < 4; ++j)
			{
				mji = s[j][i]/s[i][i];
				s[j][i] = 0.0;
				for(jj=i+1; jj<8; jj++)
					s[j][jj] -= mji*s[i][jj];
			}
	}

	if(s[3][3] == 0.0) return; // singular matrix!
	
	//
	// Now we have an upper triangular matrix.
	//
	//  x x x x | y y y y
	//  0 x x x | y y y y 
	//  0 0 x x | y y y y
	//  0 0 0 x | y y y y
	//
	//  we'll back substitute to get the inverse
	//
	//  1 0 0 0 | z z z z
	//  0 1 0 0 | z z z z
	//  0 0 1 0 | z z z z
	//  0 0 0 1 | z z z z 
	//
	
	double mij;
	for(i = 3; i > 0; --i){
		for(j = i-1; j > -1; --j){
			mij = s[j][i]/s[i][i];
			for(jj = j+1; jj < 8; ++jj){
				s[j][jj] -= mij*s[i][jj];
			}
		}
	}
	
	for(i = 0; i < 4; ++i){
		for(j = 0; j < 4; ++j){
			inv_m[IDX(i,j)] = s[i][j+4]/s[i][i];
		}
	}
}


//-----------------------------------------------------------------------------
// MARK:�l�����֐�
//-----------------------------------------------------------------------------
/*!
 * �l�����̂����Z(r = p x q)�̌v�Z
 * @param[out] r �ς̌���
 * @param[in] p,q ���̎l����
 */
static void MulQuat(double r[], const double p[], const double q[])
{
	r[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
	r[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
	r[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1];
	r[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
}
/*!
 * �l�����̏�����
 * @param[inout] r ����������l����
 */
static void InitQuat(double r[])
{
	r[0] = 1.0;
	r[1] = 0.0;
	r[2] = 0.0;
	r[3] = 0.0;
}
/*!
 * �l�����̃R�s�[(r = p)
 * @param[out] r �R�s�[��̎l����
 * @param[in]  p �R�s�[���̎l����
 */
static void CopyQuat(double r[], const double p[])
{
	r[0] = p[0];
	r[1] = p[1];
	r[2] = p[2];
	r[3] = p[3];
}

/*
 * �l���� -> ��]�ϊ��s��
 * @param[out] r ��]�ϊ��s��
 * @param[in] q �l����
 */
static void RotQuat(double r[], const double q[])
{
	double x2 = q[1]*q[1]*2.0;
	double y2 = q[2]*q[2]*2.0;
	double z2 = q[3]*q[3]*2.0;
	double xy = q[1]*q[2]*2.0;
	double yz = q[2]*q[3]*2.0;
	double zx = q[3]*q[1]*2.0;
	double xw = q[1]*q[0]*2.0;
	double yw = q[2]*q[0]*2.0;
	double zw = q[3]*q[0]*2.0;

	r[ 0] = 1.0 - y2 - z2;
	r[ 1] = xy + zw;
	r[ 2] = zx - yw;
	r[ 4] = xy - zw;
	r[ 5] = 1.0 - z2 - x2;
	r[ 6] = yz + xw;
	r[ 8] = zx + yw;
	r[ 9] = yz - xw;
	r[10] = 1.0 - x2 - y2;
	r[ 3] = r[ 7] = r[11] = r[12] = r[13] = r[14] = 0.0;
	r[15] = 1.0;
}
/*
 * ��]�ϊ��s�� -> �l����
 * @param[in] q �l����
 * @param[out] r ��]�ϊ��s��(4x4)
 */
static void QuatRot(double q[], double r[])
{
	double tr, s;
	const int nxt[3] = { 1, 2, 0 };

	tr = r[0]+r[5]+r[10];

	if(tr > 0.0){
		s = sqrt(tr+r[15]);
		q[3] = s*0.5;
		s = 0.5/s;

		q[0] = (r[6]-r[9])*s;
		q[1] = (r[8]-r[2])*s;
		q[2] = (r[1]-r[4])*s;
	}
	else{
		int i, j, k;
		i = 0;
		if(r[5] > r[0])
			i = 1;

		if(r[10] > r[IDX(i, i)])
			i = 2;

		j = nxt[i];
		k = nxt[j];

		s = sqrt((r[IDX(i, j)]-(r[IDX(j, j)]+r[IDX(k, k)]))+1.0);

		q[i] = s*0.5;
		s = 0.5/s;

		q[3] = (r[IDX(j, k)]-r[IDX(k, j)])*s;
		q[j] = (r[IDX(i, j)]+r[IDX(j, i)])*s;
		q[k] = (r[IDX(i, k)]+r[IDX(k, i)])*s;
	}
}

/*
 * ���ʐ��`���
 * @param[out] p ��Ԍ��ʂ̎l����
 * @param[in] q0,q1 ��ԑΏ�2�x�N�g��
 * @param[in] t �p�����[�^
 */
void QuatLerp(double p[], const double q0[], const double q1[], const double t)
{
	double qr = q0[0]*q1[0]+q0[1]*q1[1]+q0[2]*q1[2]+q0[3]*q1[3];
	double ss = 1.0-qr*qr;

	if(ss == 0.0){
		p[0] = q0[0];
		p[1] = q0[1];
		p[2] = q0[2];
		p[3] = q0[3];
	}
	else{
		double sp = sqrt(ss);
		double ph = acos(qr);	// �x�N�g���Ԃ̊p�x��
		double pt = ph*t;		// ��t
		double t1 = sin(pt)/sp;
		double t0 = sin(ph-pt)/sp;

		p[0] = q0[0]*t0+q1[0]*t1;
		p[1] = q0[1]*t0+q1[1]*t1;
		p[2] = q0[2]*t0+q1[2]*t1;
		p[3] = q0[3]*t0+q1[3]*t1;
	}
}

/*
 * �����̃N�H�[�^�j�I���Ԃ̋��ʐ��`��ԁi�܂���j
 *   p �� t[i] �ɂ�����N�H�[�^�j�I�� q[i], 0 <= i < n �ɑ΂���
 *        u �ɂ������Ԓl
*/
void QuatMultiLerp(double p[], const double t[], const double q[][4], const int n,
						  const double u)
{
	int i = 0, j = n-1;

	// u ���܂� t �̋�� [t[i], t[i+1]) ��񕪖@�ŋ��߂�
	while(i < j){
		int k = (i+j)/2;
		if(t[k] < u){
			i = k+1;
		}
		else{
			j = k;
		}
	}
	if(i > 0){
		--i;
	}

	QuatLerp(p, q[i], q[i+1], (u-t[i])/(t[i+1]-t[i]));
}

void SetQuat(double q[], const double ang, const double ax, const double ay, const double az)
{
	double as = ax*ax+ay*ay+az*az;
	if(as < RX_FEQ_EPS){
		q[0] = 1.0;
		q[1] = 0.0;
		q[2] = 0.0;
		q[3] = 0.0;
	}
	else{
		as = sqrt(as);
		double rd = 0.5*RX_TO_RADIANS(ang);
		double st = sin(rd);

		q[0] = cos(rd);
		q[1] = ax*st/as;
		q[2] = ay*st/as;
		q[3] = az*st/as;
	}

}

/*!
 * �l�����ɂ��x�N�g���̉�]
 * @param[out] dst ��]��̃x�N�g��
 * @param[in] q �l����
 * @param[in] src �x�N�g��
 */
inline void QuatVec(double dst[], const double q[], const double src[])
{
	double v_coef = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
	double u_coef = 2.0*(src[0]*q[1]+src[1]*q[2]+src[2]*q[3]);
	double c_coef = 2.0*q[0];

	dst[0] = v_coef*src[0]+u_coef*q[1]+c_coef*(q[2]*src[2]-q[3]*src[1]);
	dst[1] = v_coef*src[1]+u_coef*q[2]+c_coef*(q[3]*src[0]-q[1]*src[2]);
	dst[2] = v_coef*src[2]+u_coef*q[3]+c_coef*(q[1]*src[1]-q[2]*src[0]);

}

/*!
 * �����l����
 * @param[in] q ���̎l����
 * @param[out] qc �����l����
 */
inline void QuatInv(const double q[], double qc[])
{
	qc[0] =  q[0];
	qc[1] = -q[1];
	qc[2] = -q[2];
	qc[3] = -q[3];
}


//-----------------------------------------------------------------------------
// MARK:�g���b�N�{�[��
//-----------------------------------------------------------------------------
/*!
 * �R���X�g���N�^
 */
rxTrackball::rxTrackball()
{
	m_fTransScale = 10.0;

	m_iVeloc[0] = 0; m_iVeloc[1] = 0;

	// ��](�N�H�[�^�j�I��)
	InitQuat(m_fQuatRot);

	// �h���b�O���̉�]������(�N�H�[�^�j�I��)
	InitQuat(m_fQuatIncRot);

	// ��]�̕ϊ��s��
	Identity(m_fMatRot);

	// �J�����p��
	m_fTransDist[0] = 0.0; m_fTransDist[1] = 0.0;

	// �X�P�[�����O
	m_fScaleDist = 0.0;

	// �h���b�O�����ۂ�(1:���h���b�O, 2:�E�h���b�O, 3:�~�h���h���b�O)
	m_iDrag = 0;
}

/*!
 * �f�X�g���N�^
 */
rxTrackball::~rxTrackball()
{
}


/*
 * �g���b�N�{�[�������̏�����
 *  - �v���O�����̏����������̂Ƃ���Ŏ��s����
 */
void rxTrackball::Init(void)
{
	// �h���b�O���ł͂Ȃ�
	m_iDrag = 0;

	// �P�ʃN�H�[�^�[�j�I��
	InitQuat(m_fQuatIncRot);

	// ��]�s��̏�����
	RotQuat(m_fMatRot, m_fQuatRot);

	// �J�[�\�����x
	m_iVeloc[0] = 0;
	m_iVeloc[1] = 0;
}

/*
 * �g���b�N�{�[������̈�
 *  - Reshape �R�[���o�b�N (resize) �̒��Ŏ��s����
 * @param[in] w,h �r���[�|�[�g�̑傫��
 */
void rxTrackball::SetRegion(int w, int h)
{
	// �}�E�X�|�C���^�ʒu�̃E�B���h�E���̑��ΓI�ʒu�ւ̊��Z�p
	m_fW = (double)w;
	m_fH = (double)h;
}


/*
 * 3D��ԗ̈�̐ݒ�
 *  - 
 * @param[in] l ��\����
 */
void rxTrackball::SetSpace(double l)
{
	m_fTransScale = 10.0*l;
}


//-----------------------------------------------------------------------------
// MARK:�}�E�X�h���b�O
//-----------------------------------------------------------------------------
/*
 * �h���b�O�J�n
 *  - �}�E�X�{�^�����������Ƃ��Ɏ��s����
 * @param[in] x,y �}�E�X�ʒu
 */
void rxTrackball::Start(int x, int y, int button)
{
	if(x < 0 || y < 0 || x > (int)m_fW || y > (int)m_fH) return;
	//RXCOUT << "Start " << x << " x " << y << " - " << button << endl;

	// �h���b�O�J�n
	m_iDrag = button;

	// �h���b�O�J�n�_���L�^
	m_iSx = x;
	m_iSy = y;

	m_iPx = x;
	m_iPy = y;

}


void rxTrackball::TrackballLastRot(void)
{
	double dq[4];
	CopyQuat(dq, m_fQuatRot);

	// �N�H�[�^�j�I�����|���ĉ�]������
	MulQuat(m_fQuatRot, m_fQuatIncRot, dq);

	// �N�H�[�^�j�I�������]�̕ϊ��s������߂�
	RotQuat(m_fMatRot, m_fQuatRot);
}

void rxTrackball::TrackballRot(double vx, double vy)
{
	if(m_fW < RX_FEQ_EPS || m_fH < RX_FEQ_EPS) return; 
	double dx = vx/m_fW;
	double dy = vy/m_fH;
	double a = sqrt(dx*dx+dy*dy);

	// ��]����
	if(a != 0.0){
		double ar = a*TB_ROT_SCALE*0.5;
		double as = sin(ar)/a;
		double dq[4] = { cos(ar), dy*as, dx*as, 0.0 };

		m_fLastRot[0] = cos(ar);
		m_fLastRot[1] = dy*as;
		m_fLastRot[2] = dx*as;
		m_fLastRot[3] = 0.0;

		CopyQuat(m_fQuatIncRot, dq);
		CopyQuat(dq, m_fQuatRot);

		// �N�H�[�^�j�I�����|���ĉ�]������
		MulQuat(m_fQuatRot, m_fQuatIncRot, dq);

		// �N�H�[�^�j�I�������]�̕ϊ��s������߂�
		RotQuat(m_fMatRot, m_fQuatRot);
	}
	else{
		InitQuat(m_fQuatIncRot);
	}
}

/*
 * �h���b�O��
 *  - �}�E�X�̃h���b�O���Ɏ��s����
 * @param[in] x,y �}�E�X�ʒu
 */
void rxTrackball::Motion(int x, int y, bool last)
{
	if(x < 0 || y < 0 || x > (int)m_fW || y > (int)m_fH) return;
	//RXCOUT << "Motion " << x << " x " << y << endl;

	if(m_iDrag){
		if(m_iDrag == 1){		// ��]
			// �}�E�X�|�C���^�̈ʒu�̃h���b�O�J�n�ʒu����̕ψ�
			TrackballRot((double)(x-m_iPx), (double)(y-m_iPy));
		}
		else if(m_iDrag == 2){	// �p��
			// �}�E�X�|�C���^�̈ʒu�̑O��ʒu����̕ψ�
			double dx = (x-m_iPx)/m_fW;
			double dy = (y-m_iPy)/m_fH;

			m_fTransDist[0] += m_fTransScale*dx;
			m_fTransDist[1] -= m_fTransScale*dy;
		}
		else if(m_iDrag == 3){	// �X�P�[�����O
			// �}�E�X�|�C���^�̈ʒu�̑O��ʒu����̕ψ�
			double dx = (x-m_iPx)/m_fW;
			double dy = (y-m_iPy)/m_fH;

			m_fScaleDist += m_fTransScale*dy;
		}

		if(!last){
			m_iVeloc[0] = x-m_iPx;
			m_iVeloc[1] = y-m_iPy;
		}

		m_iPx = x;
		m_iPy = y;
	}
}

/*
 * ��~
 *  - �}�E�X�{�^���𗣂����Ƃ��Ɏ��s����
 * @param[in] x,y �}�E�X�ʒu
 */
void rxTrackball::Stop(int x, int y)
{
	if(x < 0 || y < 0 || x > (int)m_fW || y > (int)m_fH || !m_iDrag) return;
	//RXCOUT << "Stop " << x << " x " << y << endl;

	// �h���b�O�I���_�ɂ������]�����߂�
	TrackballRot(x-m_iPx, y-m_iPy);
	//Motion(x, y, true);

	// �h���b�O�I��
	m_iDrag = 0;
}


//-----------------------------------------------------------------------------
// MARK:�p�����[�^�擾
//-----------------------------------------------------------------------------
/*
 * ��]�̕ϊ��s���Ԃ�
 *  - �Ԓl�� glMultMatrixd() �ȂǂŎg�p���ăI�u�W�F�N�g����]����
 */
double *rxTrackball::GetRotation(void)
{
	return m_fMatRot;
}
void rxTrackball::GetQuaternion(double q[4]) const
{
	CopyQuat(q, m_fQuatRot);
}

/*
 * �X�P�[�����O�ʂ�Ԃ�
 *  - �Ԓl�� glTranslatef() �ȂǂŎg�p���Ď��_�ړ�
 */
double rxTrackball::GetScaling(void) const
{
	return m_fScaleDist;
}
void rxTrackball::GetScaling(double &s) const
{
	s = m_fScaleDist;
}

/*
 * ���s�ړ��ʂ�Ԃ�
 *  - �Ԓl�� glTranslatef() �ȂǂŎg�p���Ď��_�ړ�
 * @param[in] i 0:x,1:y
 */
double rxTrackball::GetTranslation(int i) const
{
	return m_fTransDist[i];
}
void rxTrackball::GetTranslation(double t[2]) const
{
	t[0] = m_fTransDist[0];
	t[1] = m_fTransDist[1];
}



double* rxTrackball::GetQuaternionR(void)
{
	return m_fQuatRot;
}
double* rxTrackball::GetTranslationR(void)
{
	return m_fTransDist;
}
double* rxTrackball::GetScalingR(void)
{
	return &m_fScaleDist;
}


/*!
 * �S�ϊ��s��
 * @param[out] m[16] 
 */
void rxTrackball::GetTransform(double m[16])
{
	Identity(m);
	
	for(int i = 0; i < 3; ++i){
		for(int j = 0; j < 3; ++j){
			m[IDX(i, j)] = m_fMatRot[IDX(i, j)];
		}
	}

	m[IDX(0, 3)] = m_fTransDist[0];
	m[IDX(1, 3)] = m_fTransDist[1];
	m[IDX(2, 3)] = m_fScaleDist;
}


/*
 * �g���b�N�{�[�����x
 * @param[out] vx,vy ���x(�s�N�Z��/�t���[��)
 */
void rxTrackball::GetLastVeloc(int &vx, int &vy)
{
	vx = m_iVeloc[0];
	vy = m_iVeloc[1];
}

/*!
 * �O���[�o�����W����I�u�W�F�N�g���[�J�����W�ւ̕ϊ�
 * @param[out] dst ���[�J�����W
 * @param[in]  src �O���[�o�����W
 */
void rxTrackball::CalLocalPos(double dst[4], const double src[4])
{
	// ���s�ړ�
	double pos[4];
	pos[0] = src[0]-m_fTransDist[0];
	pos[1] = src[1]-m_fTransDist[1];

	// �X�P�[�����O
	pos[2] = src[2]-m_fScaleDist;

	pos[3] = 0.0;

	// ��]
	double qc[4];
	QuatInv(m_fQuatRot, qc);
	QuatVec(dst, qc, pos);
}

/*!
 * �I�u�W�F�N�g���[�J�����W����O���[�o�����W�ւ̕ϊ�
 * @param[out] dst �O���[�o�����W
 * @param[in]  src ���[�J�����W
 */
void rxTrackball::CalGlobalPos(double dst[4], const double src[4])
{
	// ��]
	QuatVec(dst, m_fQuatRot, src);

	// ���s�ړ�
	dst[0] += m_fTransDist[0];
	dst[1] += m_fTransDist[1];

	// �X�P�[�����O
	dst[2] += m_fScaleDist;

}

/*!
 * ���_�ʒu���Z�o
 * @param[out] pos ���_�ʒu
 */
void rxTrackball::GetViewPosition(double pos[3])
{
	double src[4] = {0, 0, 0, 0};
	double dst[4];
	CalLocalPos(dst, src);
	pos[0] = dst[0];
	pos[1] = dst[1];
	pos[2] = dst[2];
}

/*!
 * �O���[�o�����W����I�u�W�F�N�g���[�J�����W�ւ̕ϊ�(��]�̂�)
 * @param[out] dst ���[�J�����W
 * @param[in]  src �O���[�o�����W
 */
void rxTrackball::CalLocalRot(double dst[4], const double src[4])
{
	// ��]
	double qc[4];
	QuatInv(m_fQuatRot, qc);
	QuatVec(dst, qc, src);
}

/*!
 * �I�u�W�F�N�g���[�J�����W����O���[�o�����W�ւ̕ϊ�(��]�̂�)
 * @param[out] dst �O���[�o�����W
 * @param[in]  src ���[�J�����W
 */
void rxTrackball::CalGlobalRot(double dst[4], const double src[4])
{
	QuatVec(dst, m_fQuatRot, src);
}

/*!
 * �����������Z�o
 * @param[out] dir ��������
 */
void rxTrackball::GetViewDirection(double dir[3])
{
	double src[4] = {0, 0, -1, 0};
	double dst[4];
	CalLocalRot(dst, src);
	dir[0] = dst[0];
	dir[1] = dst[1];
	dir[2] = dst[2];
}


/*
 * OpenGL�ɉ�]�̕ϊ��s���ݒ�
 */
void rxTrackball::ApplyRotation(void)
{
	glMultMatrixd(m_fMatRot);
}

/*
 * OpenGL�ɃX�P�[�����O��ݒ�
 */
void rxTrackball::ApplyScaling(void)
{
	glTranslated(0.0, 0.0, m_fScaleDist);
}

/*
 * OpenGL�ɕ��s�ړ���ݒ�
 */
void rxTrackball::ApplyTranslation(void)
{
	glTranslated(m_fTransDist[0], m_fTransDist[1], 0.0);
}

/*
 * OpenGL�ɉ�]�̕ϊ��s���ݒ�
 */
void rxTrackball::ApplyQuaternion(const double q[4])
{
	double m[16];
	RotQuat(m, q);
	glMultMatrixd(m);
}

/*!
 * OpenGL�̃I�u�W�F�N�g�ϊ�
 */
void rxTrackball::Apply(void)
{
	ApplyScaling();
	ApplyTranslation();
	ApplyRotation();
}


//-----------------------------------------------------------------------------
// MARK:�p�����[�^�ݒ�
//-----------------------------------------------------------------------------
/*
 * �X�P�[�����O�ʂ�ݒ�
 * @param[in] z �ړ���
 */
void rxTrackball::SetScaling(double z)
{
	m_fScaleDist = z;
}

/*
 * ���s�ړ��ʂ�ݒ�
 * @param[in] x,y �ړ���
 */
void rxTrackball::SetTranslation(double x, double y)
{
	m_fTransDist[0] = x;
	m_fTransDist[1] = y;
}

/*
 * ��]�ʂ�ݒ�
 * @param[in] rot 4x4�s��
 */
void rxTrackball::SetRotation(double rot[16])
{
	for(int i = 0; i < 16; ++i){
		m_fMatRot[i] = rot[i];

		// ��]�̕ϊ��s�񂩂�N�H�[�^�j�I�������߂�
		QuatRot(m_fQuatRot, m_fMatRot);
	}
}


/*
 * ��]�ʂ�ݒ�
 * @param[in] q �l����
 */
void rxTrackball::SetQuaternion(double q[4])
{
	// �N�H�[�^�j�I�������]�̕ϊ��s������߂�
	RotQuat(m_fMatRot, q);

	// ��]�̕ۑ�
	CopyQuat(m_fQuatRot, q);
}

/*
 * ��]�ʂ�ݒ�
 * @param[in] ang ��]��[deg]
 * @param[in] x,y,z ��]��
 */
void rxTrackball::SetRotation(double ang, double x, double y, double z)
{
	double a = sqrt(x*x+y*y+z*z);

	if(!RX_FEQ2(a, 0.0, RX_FEQ_EPS)){
		double ar = 0.5*RX_TO_RADIANS(ang);
		double as = sin(ar)/a;
		double dq[4] = { cos(ar), x*as, y*as, z*as };

		// �N�H�[�^�j�I�������]�̕ϊ��s������߂�
		RotQuat(m_fMatRot, dq);

		// ��]�̕ۑ�
		CopyQuat(m_fQuatRot, dq);
	}
}


/*
 * ��]��ǉ�
 * @param[in] ang ��]��[deg]
 * @param[in] x,y,z ��]��
 */
void rxTrackball::AddRotation(double ang, double x, double y, double z)
{
	double a = sqrt(x*x+y*y+z*z);

	if(!RX_FEQ2(a, 0.0, RX_FEQ_EPS)){
		double ar = 0.5*RX_TO_RADIANS(ang);
		double as = sin(ar)/a;
		double dq[4] = { cos(ar), x*as, y*as, z*as };

		m_fLastRot[0] = cos(ar);
		m_fLastRot[1] = x*as;
		m_fLastRot[2] = y*as;
		m_fLastRot[3] = 0.0;

		CopyQuat(m_fQuatIncRot, dq);
		CopyQuat(dq, m_fQuatRot);

		// �N�H�[�^�j�I�����|���ĉ�]������
		MulQuat(m_fQuatRot, m_fQuatIncRot, dq);

		// �N�H�[�^�j�I�������]�̕ϊ��s������߂�
		RotQuat(m_fMatRot, m_fQuatRot);

	}
}


void rxTrackball::GetLastRotation(double &ang, double &x, double &y, double &z)
{
	ang = m_fLastRot[0];
	x = m_fLastRot[1];
	y = m_fLastRot[2];
	z = m_fLastRot[3];
}


inline double dot(const double a[3], const double b[3])
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

inline void cross(const double a[3], const double b[3], double c[3])
{
	c[0] = a[1]*b[2]-a[2]*b[1];
	c[1] = a[2]*b[0]-a[0]*b[2];
	c[2] = a[0]*b[1]-a[1]*b[0];
}

inline double normalize(double a[3])
{
	double d = dot(a, a);
	if(d > 0.0){
		double l = sqrt(1.0/d);
		a[0] *= l;
		a[1] *= l;
		a[2] *= l;
	}
	return d;
}



/*!
 * �}�E�X�N���b�N�����ʒu�ւ̕����x�N�g����Ԃ�
 * @param[in] 
 * @param[out] 
 * @return 
 */
void rxTrackball::GetRayTo(int x, int y, double fov, double ray_to[3])
{
	double eye_pos[3], eye_dir[3], up_dir[3] = {0, 1, 0};
	double init_pos[3] = {0, 0, 0};
	double init_dir[3] = {0, 0, -1};
	double init_up[3] = {0, 1, 0};
	CalLocalPos(eye_pos, init_pos);
	CalLocalRot(eye_dir, init_dir);
	CalLocalRot(up_dir,  init_up);

	normalize(eye_dir);

	double far_d = 10.0;
	eye_dir[0] *= far_d;
	eye_dir[1] *= far_d;
	eye_dir[2] *= far_d;

	normalize(up_dir);

	// �����ɐ����ȕ��ʂ̍��E�C�㉺����
	double hor[3], ver[3];
	cross(eye_dir, up_dir, hor);
	normalize(hor);
	cross(hor, eye_dir, ver);
	normalize(ver);

	double tanfov = tan(0.5*RX_TO_RADIANS(fov));

	double d = 2.0*far_d*tanfov;
	double aspect = m_fW/m_fH;
	hor[0] *= d*aspect;
	hor[1] *= d*aspect;
	hor[2] *= d*aspect;
	ver[0] *= d;
	ver[1] *= d;
	ver[2] *= d;

	// �`�敽�ʂ̒��S
	ray_to[0] = eye_pos[0]+eye_dir[0];
	ray_to[1] = eye_pos[1]+eye_dir[1];
	ray_to[2] = eye_pos[2]+eye_dir[2];

	// ���S���王�_�ɐ����ȕ��ʂ̍��E�C�㉺�����Ƀ}�E�X�̍��W���ړ�������
	double dx = (x-0.5*m_fW)/m_fW, dy = (0.5*m_fH-y)/m_fH;
	ray_to[0] += dx*hor[0];
	ray_to[1] += dx*hor[1];
	ray_to[2] += dx*hor[2];
	ray_to[0] += dy*ver[0];
	ray_to[1] += dy*ver[1];
	ray_to[2] += dy*ver[2];
}
