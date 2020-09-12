/*! 
  @file rx_bitmap.h
	
  @brief �r�b�g�}�b�v�t�@�C���ǂݍ��݁C�����o��
	- http://ja.wikipedia.org/wiki/Windows_bitmap

	- OS/2(V1), Windows(V3,V4,V5)�̔񈳏k�r�b�g�}�b�v�ɑΉ�
	- �r�b�g�t�B�[���h�t���r�b�g�}�b�v�C�J���[�p���b�g�t���r�b�g�}�b�v�ɑΉ�

	- RLE, JPEG, PNG���k�͖��Ή�
	- V4,V5�̃J���[�}�l�W�����g�C�v���t�@�C���ɂ͖��Ή�
 
  @author Makoto Fujisawa
  @date 2011-06
*/
// FILE --rx_bitmap.h--

#ifndef _RX_BITMAP_H_
#define _RX_BITMAP_H_


//-----------------------------------------------------------------------------
// �C���N���[�h�t�@�C��
//-----------------------------------------------------------------------------
#include <cstdio>

#include <vector>
#include <string>


//-----------------------------------------------------------------------------
// ��`
//-----------------------------------------------------------------------------
enum rxBitmapType
{
	RX_BMP_OS2_V1, 
	RX_BMP_OS2_V2, 
	RX_BMP_WINDOWS_V3, 
	RX_BMP_WINDOWS_V4, 
	RX_BMP_WINDOWS_V5, 
};

//-----------------------------------------------------------------------------
// �\����
//-----------------------------------------------------------------------------
//! ICM(Image Color Management)��` (36�r�b�g,CIEXYZTRIPLE�^�Ɠ���)
//  - http://www.adobe.com/jp/support/techguides/color/colormodels/ciexyz.html
struct rxICMColor
{
	long red[3];	//!< R�px,y,z
	long green[3];	//!< G�px,y,z
	long blue[3];	//!< B�px,y,z
};

//! �J���[�p���b�g�pRGB�\����
struct rxBmpRGB
{
	unsigned char r, g, b;	//!< �ԁC�΁C����
};

//! �J���[�p���b�g�pRGB�\����(�\��̈�܂�)
struct rxBmpRGB4
{
	unsigned char r, g, b;	//!< �ԁC�΁C����
	unsigned char res;		//!< �\��̈�
};

//! �t�@�C���w�b�_ (14�o�C�g-2�o�C�g)
// - http://msdn.microsoft.com/en-us/library/dd183374(VS.85).aspx
struct rxBmpFileHeader
{
	//char type[2];			//!< �t�@�C���^�C�v(2�o�C�g) - BMP�Ȃ�'B''M'
	unsigned int size;		//!< �t�@�C���T�C�Y(4�o�C�g)
	char reserved[4];		//!< �\��̈�1,2(2�o�C�g x 2)
	unsigned int offset;	//!< �C���[�W�f�[�^�܂ł̃I�t�Z�b�g(4�o�C�g) - INFO�Ȃ�54
};

//! ���w�b�_(CORE�^�C�v(OS/2),12�o�C�g)
// - http://msdn.microsoft.com/en-us/library/dd183372(VS.85).aspx
struct rxBmpInfoHeaderOS2
{
	unsigned int size;			//!< �w�b�_�T�C�Y(4�o�C�g) - 12�o�C�g
	unsigned short width;		//!< �摜�̕�(2�o�C�g) - �s�N�Z����,1�ȏ�
	unsigned short height;		//!< �摜�̍���(2�o�C�g) - �s�N�Z����,1�ȏ�
	unsigned short planes;		//!< �摜�̃`�����l����(2�o�C�g) - 1
	unsigned short bitcount;	//!< �r�b�g��/�s�N�Z��(2�o�C�g) - 1,4,8,24
};

//! ���w�b�_(INFO,V4,V5�^�C�v(Windows),40,108,124�o�C�g)
// - INFO : http://msdn.microsoft.com/en-us/library/dd183376(v=VS.85).aspx
// - V4   : http://msdn.microsoft.com/en-us/library/dd183380(VS.85).aspx
// - V5   : http://msdn.microsoft.com/en-us/library/dd183381(VS.85).aspx
struct rxBmpInfoHeaderWin
{
	unsigned int size;			//!< �w�b�_�T�C�Y(4�o�C�g) - 40,108,124�o�C�g
	int width;					//!< �摜�̕�(4�o�C�g) - �s�N�Z����,1�ȏ�
	int height;					//!< �摜�̍���(4�o�C�g) - �s�N�Z����,0�ȊO,�}�C�i�X�̏ꍇ�̓g�b�v�_�E���`��
	unsigned short planes;		//!< �摜�̃`�����l����(2�o�C�g) - 1
	unsigned short bitcount;	//!< �r�b�g��/�s�N�Z��(2�o�C�g) - 0,1,4,8,16,24,32
	unsigned int compression;	//!< ���k�`��(4�o�C�g) - 0(�񈳏k), 1(8�r�b�gRLE), 2(4�r�b�gRLE), 3(�r�b�g�t�B�[���h�t���񈳏k), 4(JPEG), 5(PNG)
	unsigned int image_size;	//!< �摜�f�[�^�̃T�C�Y(4�o�C�g) - �o�C�g��
	unsigned int xpixel;		//!< �����𑜓x(4�o�C�g) - �s�N�Z����/���[�g��
	unsigned int ypixel;		//!< �����𑜓x(4�o�C�g) - �s�N�Z����/���[�g��
	unsigned int num_color_idx;		//!< �g�p����F��(4�o�C�g) - �J���[�p���b�g�Ɋi�[�����F��
	unsigned int num_important_idx;	//!< �d�v�ȐF��(4�o�C�g) - �J���[�p���b�g�̏d�v�F�̐�(�\���ɕK�v�Ȑ�)

	// V4,V5�^�C�v�p
	unsigned int red_mask;		//!< �Ԑ����̃J���[�}�X�N(4�o�C�g)
	unsigned int green_mask;	//!< �ΐ����̃J���[�}�X�N(4�o�C�g)
	unsigned int blue_mask;		//!< �����̃J���[�}�X�N(4�o�C�g)
	unsigned int alpha_mask;	//!< �A���t�@�����̃J���[�}�X�N(4�o�C�g)
	unsigned int color_space;	//!< �F���(4�o�C�g) - 0
	rxICMColor icm;				//!< CIE XYZ(36�o�C�g)
	//CIEXYZTRIPLE icm;
	unsigned int red_gamma;		//!< �Ԑ����̃K���}�l(4�o�C�g)
	unsigned int green_gamma;	//!< �ΐ����̃K���}�l(4�o�C�g)
	unsigned int blue_gamma;	//!< �����̃K���}�l(4�o�C�g)

	// V5�^�C�v�p
	unsigned int intent;		//!< sRGB�F��ԃ^�C�v(4�o�C�g) - ICC32���� : 1(Saturation),2(Relative Colorimetric),4(Perceptual),8(Absolute Colorimetic)
	unsigned int profile_data;	//!< �v���t�@�C���f�[�^�̃I�t�Z�b�g(4�o�C�g) - ���w�b�_�̐擪����̃I�t�Z�b�g�o�C�g��
	unsigned int profile_size;	//!< �v���t�@�C���f�[�^�̃T�C�Y(4�o�C�g) - �o�C�g��
	unsigned int reserved;		//!< �\��̈�(4�o�C�g) - 0
};

//! �r�b�g�t�B�[���h(12�o�C�g)
// - INFO�^�C�v��bitcount��16��32�Ccompression��3�̏ꍇ�ɏ��w�b�_�̒���ɑ���
struct rxBmpBitField
{
	unsigned int red_mask;		//!< �Ԑ����̃J���[�}�X�N(4�o�C�g)
	unsigned int green_mask;	//!< �ΐ����̃J���[�}�X�N(4�o�C�g)
	unsigned int blue_mask;		//!< �����̃J���[�}�X�N(4�o�C�g)
};


//-----------------------------------------------------------------------------
// �Œ菬���_���̕ϊ��֐�(�K���}�l�p)
//-----------------------------------------------------------------------------
/*!
 * ���������_��(float)����Œ菬���_���ւ̕ϊ�
 *  - �K���}�l�i�[�p
 *  - 8.8�Œ菬���_��
 *  - 9�`16�r�b�g��������
 * @param[in] x ���������_��
 * @param[in] shift �Œ菬���_���̏���������
 * @return �Œ菬���_��
 */
inline int FloatToFixForGamma(float x)
{
	int power = 1 << 16;
	return (int)(power*x) & 0x00ffff00;
}

/*!
 * �Œ菬���_�����畂�������_��(float)�ւ̕ϊ�
 * @param[in] x �Œ菬���_��
 * @param[in] shift �Œ菬���_���̏���������
 * @return ���������_��
 */
inline float FixToFloatForGamma(int x)
{
	int power = 1 << 16;
	return (float)x/(float)power;
}


//-----------------------------------------------------------------------------
// BMP�t�@�C���̓ǂݍ��݂Ə�������
//-----------------------------------------------------------------------------
/*!
 * BMP�t�@�C���̓ǂݍ���
 * @param[in] fn �t�@�C����
 * @param[out] w,h �摜�T�C�Y
 * @param[out] c �摜�̐F�[�x
 * @return �摜�f�[�^
 */
static unsigned char* ReadBitmapFile(const std::string &fn, int &w, int &h, int &c)
{
	// �t�@�C�����o�C�i�����[�h�ŊJ��
	FILE *fp;
	if((fp = fopen(fn.c_str(), "rb")) == NULL){
		fprintf(stderr, "bitmap error : cannot open %s file\n", fn.c_str());
		return 0;
	}

	//fseek(fp, 0L, SEEK_SET);

	// �t�@�C���w�b�_�̓ǂݍ���
	char type[2];
	rxBmpFileHeader file_header;
	fread(type, sizeof(char), 2, fp);						// 2�o�C�g
	fread(&file_header, sizeof(file_header), 1, fp);		// 12�o�C�g

	// ���w�b�_�T�C�Y�̓ǂݍ���
	unsigned int info_header_size = 0;
	fread(&info_header_size, sizeof(unsigned int), 1, fp);	// 4�o�C�g
	fseek(fp, (long)(sizeof(char)*14), SEEK_SET);

	int bitcount = 0;	// �s�N�Z���r�b�g��
	
	std::vector<rxBmpRGB4> cpalette;	// �J���[�p���b�g
	bool use_cmask = false;		// �J���[�}�X�N�g�p�t���O
	unsigned int cmask[4] = {0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000};	// �J���[�}�X�N(RGBA)
	int cmask_shift = 0;		// �J���[�}�X�N�̃r�b�g��

	// ���w�b�_�̓ǂݍ���
	if(info_header_size == 12 || info_header_size == 64){	// OS/2
		//printf("OS/2\n");
		rxBmpInfoHeaderOS2 info_header;
		fread(&info_header, sizeof(char), info_header_size, fp);

		w = (int)info_header.width;
		h = (int)info_header.height;
		c = (int)ceil((double)info_header.bitcount/8.0);
		bitcount = (int)info_header.bitcount;

		// �J���[�p���b�g
		if(bitcount <= 8){
			std::vector<rxBmpRGB> cpalette3;
			int n = 1 << bitcount;
			cpalette3.resize(n);
			fread(&cpalette3[0], sizeof(rxBmpRGB), n, fp);

			cpalette.resize(n);
			for(int i = 0; i < n; ++i){
				cpalette[i].r = cpalette3[i].r;
				cpalette[i].g = cpalette3[i].g;
				cpalette[i].b = cpalette3[i].b;
				cpalette[i].res = 0;
			}
		}
	}
	else if(info_header_size >= 40){	// Windows
		//printf("Windows\n");
		rxBmpInfoHeaderWin info_header;
		fread(&info_header, sizeof(char), info_header_size, fp);	

		w = (int)info_header.width;
		h = (int)info_header.height;
		c = (int)ceil((double)info_header.bitcount/8.0);
		bitcount = (int)info_header.bitcount;

		// �r�b�g�t�B�[���h
		if(info_header.compression == 3 && (bitcount == 16 || bitcount == 32)){
			// �}�X�N�l�����߂�
			use_cmask = true;
			if(info_header_size == 40){
				fread(cmask, sizeof(unsigned int), 3, fp);
			}
			else{
				cmask[0] = info_header.red_mask;
				cmask[1] = info_header.green_mask;
				cmask[2] = info_header.blue_mask;
				cmask[3] = info_header.alpha_mask;
			}
			// �}�X�N�̃r�b�g��(�����̃}�X�N��1�̃r�b�g�̐��𐔂���)
			cmask_shift = (int)(log((double)(cmask[2]+1))/log(2.0));
		}

		// �J���[�p���b�g
		if(bitcount >= 1 && bitcount <= 8 && info_header.num_color_idx){
			std::vector<rxBmpRGB4> cpalette;
			int n = info_header.num_color_idx;
			cpalette.resize(n);
			fread(&cpalette[0], sizeof(rxBmpRGB4), n, fp);
		}
	}
	else{
		fclose(fp);
		return 0;
	}

	bool flip = true;
	if(h < 0){
		// �g�b�v�_�E���`��(�摜���ォ�牺�֋L�^)�̏ꍇ
		h = -h;
		flip = false;
	}
	
	if(!cpalette.empty() || use_cmask){
		c = 3;
	}

	// �o�̓f�[�^
	unsigned char* img = new unsigned char[w*h*c];

	if(!cpalette.empty()){
		// �f�[�^���擾���āC�J���[�p���b�g��K�����C�摜�f�[�^���쐬
		int bitmask = (1 << bitcount)-1;
		int div = 8/bitcount;

		int m = (w*h)/div;	// �e�s�N�Z���̃r�b�g����8�r�b�g(1�o�C�g)�Ƃ����Ƃ��̑��s�N�Z����
		unsigned char* bit_buf = new unsigned char[m];
		fread(bit_buf, sizeof(unsigned char), (size_t)(long)(m), fp);

		unsigned char* buf = bit_buf;
		for(int i = 0; i < m; ++i){
			unsigned char b = *buf;
			for(int j = 0; j < div; ++j){
				int k = (b >> bitcount*(div-j-1)) & bitmask;
				rxBmpRGB4 c = cpalette[k];
				int idx = 3*(div*i+j);
				img[idx+0] = c.r;
				img[idx+1] = c.g;
				img[idx+2] = c.b;
			}
			buf++;
		}

		free(bit_buf);
	}
	else if(use_cmask){
		// �f�[�^���擾���āC�r�b�g�t�B�[���h��K�p���C�摜�f�[�^���쐬
		// bitcount��16��32
		int bitmask = (bitcount == 16 ? 0x0000ffff : 0xffffffff);
		int div = 32/bitcount;
		int m = (w*h)/div;	// �e�s�N�Z���̃r�b�g����32�r�b�g(4�o�C�g)�Ƃ����Ƃ��̑��s�N�Z����
		unsigned int* bit_buf = new unsigned int[m];
		fread(bit_buf, sizeof(unsigned int), (size_t)(long)(m), fp);

		unsigned int* buf = bit_buf;
		for(int i = 0; i < m; ++i){
			unsigned int b = *buf;
			for(int j = 0; j < div; ++j){
				unsigned int c = (b >> bitcount*(div-j-1)) & bitmask;
				int idx = 3*(div*i+j);
				img[idx+0] = (cmask[0] & c) >> cmask_shift*2;
				img[idx+1] = (cmask[1] & c) >> cmask_shift;
				img[idx+2] = (cmask[2] & c);

			}

			buf++;
		}

		free(bit_buf);
	}
	else{
		// �摜�f�[�^�̎擾
		fread(img, sizeof(unsigned char), (size_t)(long)(w*h*c), fp);
	}


	// BGR -> RGB
	for(int j = 0; j < h; ++j){
		for(int i = 0; i < w; ++i){
			int idx = 3*(i+j*w);
			unsigned char tmp = img[idx+0];
			img[idx+0] = img[idx+2];
			img[idx+2] = tmp;
		}
	}

	// �㉺���]
	if(flip){
		int stride = w*3;
		for(int j = 0; j < h/2; ++j){
			for(int i = 0; i < stride; ++i){
				unsigned char tmp = img[j*stride+i];
				img[j*stride+i] = img[(h-j-1)*stride+i];
				img[(h-j-1)*stride+i] = tmp;
			}
		}
	}

	fclose(fp);
	return img;
}

/*!
 * BMP�t�@�C���̏�������(INFO�^�C�v)
 * @param[in] fn �t�@�C����
 * @param[in] img �摜�f�[�^
 * @param[in] w,h �摜�T�C�Y
 * @param[in] c �摜�̐F�[�x
 * @param[in] type �r�b�g�}�b�v�`��
 */
static int WriteBitmapFile(const std::string &fn, unsigned char *img, int w, int h, int c, int type = RX_BMP_WINDOWS_V3)
{
	// �t�@�C�����o�C�i�����[�h�ŊJ��
	FILE *fp;
	if((fp = fopen(fn.c_str(), "wb")) == NULL){
		fprintf(stderr, "bitmap error : cannot open %s file\n", fn.c_str());
		return 0;
	}

	// �t�@�C���w�b�_
	char file_type[2] = {'B', 'M'};
	fwrite(file_type, sizeof(char), 2, fp);

	rxBmpFileHeader file_header;
	file_header.size = w*h*c+54;
	file_header.reserved[0] = 0;
	file_header.reserved[1] = 0;
	file_header.reserved[2] = 0;
	file_header.reserved[3] = 0;
	file_header.offset = 54;

	fwrite(&file_header, sizeof(file_header), 1, fp);

	if(type == RX_BMP_OS2_V1 || type == RX_BMP_OS2_V2){
		// ���w�b�_(OS/2)
		rxBmpInfoHeaderOS2 info_header;
		info_header.size = 12;
		info_header.width = (unsigned short)w;
		info_header.height = (unsigned short)h;
		info_header.planes = 1;
		info_header.bitcount = 8*c;

		fwrite(&info_header, sizeof(info_header), 1, fp);	// 12�o�C�g
	}
	else if(type >= RX_BMP_WINDOWS_V3){
		// ���w�b�_(Windows)
		rxBmpInfoHeaderWin info_header;
		info_header.width = w;
		info_header.height = h;
		info_header.planes = 1;
		info_header.bitcount = 8*c;
		info_header.compression = 0;	// �񈳏k
		info_header.image_size = w*h*c;
		info_header.xpixel = 0;
		info_header.ypixel = 0;
		info_header.num_color_idx = 0;
		info_header.num_important_idx = 0;

		if(type == RX_BMP_WINDOWS_V3){
			info_header.size = 40;
		}
		else if(type >= RX_BMP_WINDOWS_V4){
			info_header.size = 108;

			// �J���[�}�X�N
			info_header.red_mask   = 0xff0000;
			info_header.green_mask = 0x00ff00;	
			info_header.blue_mask  = 0x0000ff;	
			info_header.alpha_mask = 0x000000;	

			// �F���
			info_header.color_space = 0;

			// ICM
			rxICMColor icm;
			icm.red[0] = icm.red[1] = icm.red[2] = 0;
			icm.green[0] = icm.green[1] = icm.green[2] = 0;
			icm.blue[0] = icm.blue[1] = icm.blue[2] = 0;
			info_header.icm = icm;

			// �K���}�l
			float gamma = 1.1f;
			info_header.red_gamma = FloatToFixForGamma(gamma);
			info_header.green_gamma = FloatToFixForGamma(gamma);
			info_header.blue_gamma = FloatToFixForGamma(gamma);

			if(type == RX_BMP_WINDOWS_V5){
				info_header.size = 124;

				// V5�^�C�v�p
				info_header.intent = 1;
				info_header.profile_data = 0;
				info_header.profile_size = 0;
				info_header.reserved = 0;
			}
		}
		
		fwrite(&info_header, sizeof(char), info_header.size, fp);
	}
	else{
		return 0;
	}


	// �摜�f�[�^
	unsigned char *img_buf = new unsigned char[w*h*c];

	// RGB -> BGR �� �㉺���]
	for(int j = 0; j < h; ++j){
		for(int i = 0; i < w; ++i){
			int idx0 = c*(i+j*w);
			int idx1 = c*(i+(h-j-1)*w);
			img_buf[idx0+0] = img[idx1+2];
			img_buf[idx0+1] = img[idx1+1];
			img_buf[idx0+2] = img[idx1+0];
		}
	}

	fwrite(img_buf, sizeof(unsigned char), (size_t)(long)(w*h*c), fp);

	delete [] img_buf;

	fclose(fp);

	return 1;
}


#endif // #ifndef _RX_BITMAP_H_
