/*! 
  @file rx_timer.h
	
  @brief ���Ԍv��
 
  @author Makoto Fujisawa
  @date 2012-02
*/
// FILE -- rx_timer.h --

#ifndef _RX_TIMER_H_
#define _RX_TIMER_H_


//-----------------------------------------------------------------------------
// �C���N���[�h�t�@�C��
//-----------------------------------------------------------------------------
#include <iostream>
#include <sstream>

#include <vector>
#include <map>
#include <string>

#define RX_USE_QPC	// Windows�ł̎��Ԍv����QueryPerformanceCounter��p����
//#define RX_USE_MM
#ifdef WIN32
	#include <windows.h>
 
	#ifdef RX_USE_MM
	#include <mmsystem.h>
	#pragma comment (lib, "winmm.lib")
	#endif
#else
	#include <ctime>
#endif
 
#ifdef WIN32

#ifdef RX_USE_QPC
	// QueryPerformanceCounter�Ńx�[�X�N���b�N��CPU���g��ꃋ�ꍇ�C���g�����ς�CPU(TurboBoost�Ȃ�)���g���Ă����
	// �v�����̕��׏�Ԃɂ���ē�����l���ω����Ă��܂��̂Œ��ӁD
	// Sleep(1000);�ȂǂŌv���l������ł��炩���߃`�F�b�N���Ă������ƁI
	// Core i7 3.2GHz, Win7 x64���ł�QueryPerformanceFrequency��3124873�ƂȂ����̂ŁCCPU�N���b�N�ł͂Ȃ��͗l�D
	#define RXTIME LONGLONG
	inline RXTIME RX_GET_TIME(void)
	{
		LARGE_INTEGER t;
		QueryPerformanceCounter((LARGE_INTEGER*)&t);
		return t.QuadPart;
	}
	inline double RX_GET_TIME2SEC(void)
	{
		LARGE_INTEGER f;
		QueryPerformanceFrequency((LARGE_INTEGER*)&f);
		return 1.0/(double)(f.QuadPart);
	}
#else
	#ifdef RX_USE_MM
		#define RXTIME DWORD
		inline RXTIME RX_GET_TIME(void){ return timeGetTime(); }
		inline double RX_GET_TIME2SEC(void){ return 1.0e-3; }
	#else
		#define RXTIME DWORD
		inline RXTIME RX_GET_TIME(void){ return GetTickCount(); }
		inline double RX_GET_TIME2SEC(void){ return 1.0e-3; }
	#endif
#endif

#else
	#define RXTIME clock_t
	inline RXTIME RX_GET_TIME(void){ return clock(); }
	inline double RX_GET_TIME2SEC(void){ return (1.0/CLOCKS_PER_SEC); }
#endif
 

using namespace std;



//-----------------------------------------------------------------------------
// ���Ԍv���N���X
//-----------------------------------------------------------------------------
class rxTimer
{
	RXTIME m_tStart, m_tEnd;
	vector<double> m_vTimes;		//!< �v�����ꂽ����
	vector<string> m_vComments;		//!< �e�v���f�[�^�̃��x�� (Split, Stop�̈����Ŏw��)
	double m_fT2S;					//!< �v�����ꂽ���ԒP�ʂ���b�P�ʂւ̕ϊ��W��
 
public:
	//! �R���X�g���N�^
	rxTimer()
	{
		m_fT2S = RX_GET_TIME2SEC();
	}
 
	//! �f�X�g���N�^
	~rxTimer(){}
 
	//! �v���J�n
	void Start(void)
	{
		m_tStart = RX_GET_TIME();
	}
 
	//! �v��
	void Split(const string &cmnt = "", bool restart = false)
	{
		m_tEnd = RX_GET_TIME();
 
		double time = (double)(m_tEnd-m_tStart)*m_fT2S;
		m_vTimes.push_back(time);
		m_vComments.push_back(cmnt);
 
		if(restart) m_tStart = RX_GET_TIME();
	}
 
	//! �v���I��
	void Stop(const string &cmnt = "")
	{
		m_tEnd = RX_GET_TIME();
 
		double time = (double)(m_tEnd-m_tStart)*m_fT2S;
		m_vTimes.push_back(time);
		m_vComments.push_back(cmnt);
 
		m_tStart = m_tEnd = 0;
	}
 
	//! ���Z�b�g
	void Reset(void)
	{
		m_vTimes.clear();
		m_vComments.clear();
		m_tStart = m_tEnd = 0;
	}
 
	// �Ō�ɋL�^���ꂽ���Ԃ��폜
	void PopBackTime(void)
	{
		m_vTimes.pop_back();
		m_vComments.pop_back();
	}
 
	//! ���Ԃ��Z�b�g(���̌v�����@�Ōv���������ʂȂ�)
	void SetTime(const double &t, const string &cmnt = "")
	{
		m_vTimes.push_back(t);
		m_vComments.push_back(cmnt);
	}
 
	//! ���Ԃ̎擾
	double GetTime(int i)
	{
		if(i >= (int)m_vTimes.size()) return 0.0;
 
		return m_vTimes[i];
	}
 
	//! �L�^���ꂽ���Ԑ��̎擾
	int GetTimeNum(void)
	{
		return (int)m_vTimes.size();
	}
 
	//! �L�^���ꂽ���Ԃ���ʏo��
	double Print(void)
	{
		int m = 0, mi;
		if(m_vTimes.empty()){
			return 0.0;
		}
		else{
			// ���v�����Ԃ��v�Z
			double total = 0.0;
			for(int i = 0; i < (int)m_vTimes.size(); ++i){
				mi = (int)m_vComments[i].size();
				if(mi > m) m = mi;
 
				total += m_vTimes[i];
			}
 
			SetTime(total, "total");
		}
 
		int cur_p = cout.precision();
		cout.precision(3);
		cout.setf(ios::fixed);
		for(int i = 0; i < (int)m_vTimes.size(); ++i){
			string spc;
			for(int k = 0; k < m-(int)m_vComments[i].size(); ++k) spc += " ";
			cout << m_vComments[i] << spc << " : " << m_vTimes[i] << endl;
		}
		cout.unsetf(ios::fixed);
		cout.precision(cur_p);
 
		double t = m_vTimes.back();
 
		PopBackTime(); // �i�[�������v���Ԃ����̌v�Z�ɔ����č폜
 
		return t;
	}
 
	//! �L�^���ꂽ���Ԃ𕶎���ɏo��
	double PrintToString(string &str)
	{
		int m = 0, mi;
		if(m_vTimes.empty()){
			return 0.0;
		}
		else{
			// ���v�����Ԃ��v�Z
			double total = 0.0;
			for(int i = 0; i < (int)m_vTimes.size(); ++i){
				mi = (int)m_vComments[i].size();
				if(mi > m) m = mi;
 
				total += m_vTimes[i];
			}
 
			SetTime(total, "total");
		}
 
		stringstream ss;
		ss.precision(3);
		ss.setf(ios::fixed);
		
		int n = (int)m_vTimes.size();
		for(int i = 0; i < n; ++i){
			string spc;
			for(int k = 0; k < m-(int)m_vComments[i].size(); ++k) spc += " ";
			ss << m_vComments[i] << spc << " : " << m_vTimes[i] << "\n";
		}
 
		ss << "\n";
		str = ss.str();
 
		double t = m_vTimes.back();
 
		PopBackTime(); // �i�[�������v���Ԃ����̌v�Z�ɔ����č폜
 
		return t;
	}
};


//! ���ώ��Ԍv���N���X
class rxTimerAvg
{
public:
	// ���Ԃƌv����
	struct rxTimeAndCount
	{
		double time;
		int count;
		int idx;
	};

	// ���Ԃƌv���񐔂𕶎���Ɗ֘A�Â���}�b�v
	typedef map<string, rxTimeAndCount> RXMAPTC;

private:
	rxTimer m_Tmr;		//!< ���Ԍv���N���X
	RXMAPTC m_TimeMap;	//!< ���Ԃƌv���񐔂𕶎���Ɗ֘A�Â���}�b�v

public:
	//! �R���X�g���N�^
	rxTimerAvg()
	{
		Clear();
		ResetTime();
		ClearTime();
	}

	/*!
	 * ���ׂăN���A
	 */
	void Clear(void)
	{
		m_TimeMap.clear();
	}

	/*!
	 * �~�ώ��Ԃ̏�����
	 */
	void ClearTime(void)
	{
		for(RXMAPTC::iterator it = m_TimeMap.begin(); it != m_TimeMap.end(); ++it){
			it->second.time = 0.0;
			it->second.count = 0;
			//it->second.idx = -1;
		}
	}

	/*!
	 * ���Z�b�g
	 */
	void ResetTime(void)
	{
		m_Tmr.Reset();
		m_Tmr.Start();
	}

	/*!
	 * �v��
	 * @param[in] cmnt ���Ԓ~�ϗp�̖��O
	 */
	void Split(const string &cmnt)
	{
		RXMAPTC::iterator i = m_TimeMap.find(cmnt);
		
		m_Tmr.Stop();
		if(i == m_TimeMap.end()){
			m_TimeMap[cmnt].time = m_Tmr.GetTime(0);
			m_TimeMap[cmnt].count = 1;
			m_TimeMap[cmnt].idx = m_TimeMap.size()-1;
		}
		else{
			m_TimeMap[cmnt].time += m_Tmr.GetTime(0);
			m_TimeMap[cmnt].count++;
		}
		m_Tmr.Reset();
		m_Tmr.Start();
	}

	/*!
	 * �����Ԃ̎擾
	 * @return ������
	 */
	double GetTotalTime(void)
	{
		if(m_TimeMap.empty()){
			m_Tmr.Stop();
			return m_Tmr.GetTime(0);
		}
		else{
			double total = 0.0;
			for(RXMAPTC::iterator it = m_TimeMap.begin(); it != m_TimeMap.end(); ++it){
				total += it->second.time/it->second.count;
			}

			return total;
		}
	}

	/*!
	 * �L�^���ꂽ���Ԃ���ʏo��
	 */
	void Print(void)
	{
		int m = 0, mi;
		double total = 0.0;
		for(RXMAPTC::iterator it = m_TimeMap.begin(); it != m_TimeMap.end(); ++it){
			mi = (int)it->first.size();
			if(mi > m) m = mi;

			total += it->second.time/it->second.count;
		}

		int cur_p = cout.precision();
		cout.precision(3);
		cout.setf(ios::fixed);
		for(RXMAPTC::iterator it = m_TimeMap.begin(); it != m_TimeMap.end(); ++it){
			string spc;
			for(int k = 0; k < m-(int)(it->first.size()); ++k) spc += " ";
			cout << it->first << spc << " : " << (it->second.time/it->second.count) << "[s]" << endl;
		}
		cout.unsetf(ios::fixed);
		cout.precision(cur_p);

		string spc;
		for(int k = 0; k < m-5; ++k) spc += " ";
		cout << "total" << spc << " : " << total << endl;
	}

	/*!
	 * �L�^���ꂽ���Ԃ𕶎���ɏo��
	 * @param[out] str �o�͕�����
	 */
	void PrintToString(string &str)
	{
		int m = 0, mi;
		double total = 0.0;
		for(RXMAPTC::iterator it = m_TimeMap.begin(); it != m_TimeMap.end(); ++it){
			mi = (int)it->first.size();
			if(mi > m) m = mi;

			total += it->second.time/it->second.count;
		}

		stringstream ss;
		ss.precision(3);
		ss.setf(ios::fixed);
		for(int i = 0; i < (int)m_TimeMap.size(); ++i){
			RXMAPTC::iterator it = m_TimeMap.begin();

			for(it = m_TimeMap.begin(); it != m_TimeMap.end(); ++it){
				if(it->second.idx == i){
					break;
				}
			}

			string spc;
			for(int k = 0; k < m-(int)(it->first.size()); ++k) spc += " ";

			ss << it->first << spc << " : " << (it->second.time/it->second.count) << "[s]\n";
		}

		string spc;
		for(int k = 0; k < m-5; ++k) spc += " ";
		ss << "total" << spc << " : " << total << "[s]\n";

		str = ss.str();
	}
};




#endif // #ifdef _RX_TIMER_H_