// MlpGuiDlg.h : header file
//

#pragma once

//-------------------------------------------------------------------------------------------------
#include "PlannerServiceHandler.h"
#include "PlannerHelper.h"
#include "../../Common/NetConfig.h"
#include <thrift/concurrency/ThreadManager.h>
#include <thrift/concurrency/PosixThreadFactory.h>
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TTransportUtils.h>
#include <thrift/TToString.h>

#include "Geometry.h"

#include <iostream>
#include <stdexcept>
#include <sstream>

using namespace std;
using namespace apache::thrift;
using namespace apache::thrift::concurrency;
using namespace apache::thrift::protocol;
using namespace apache::thrift::transport;
using namespace apache::thrift::server;

using namespace afarcloud;
using namespace geon;

//-------------------------------------------------------------------------------------------------

// CMlpGuiDlg dialog
class CMlpGuiDlg : public CDialog
{
// Construction
public:
	CMlpGuiDlg(LPTSTR lpCmdLine, CWnd* pParent = nullptr);	// standard constructor

	TSimpleServer *GetServer() const { return m_pServer; }
	int GetPort() const { return m_port; }

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_MLPGUI_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

private:
	void StartMlpServer();
	HANDLE m_hMlpServerThread;
	TSimpleServer *m_pServer;
	int m_port;

	void ClearData();
	CRect m_rcClient;
	std::vector<CPoint> m_ptHome;
	std::vector<CRect> m_rcObstacles;
	std::vector<CPoint> m_ptVehicles;
	std::vector<CRect> m_rcTasks;
	std::vector<std::vector<CPoint>> m_ptPaths;
	std::vector<CPoint> m_ptPath;
	CRect m_rcRandArea;
	CRect m_rcNavArea;

	BOOL m_bRefAreaReceived;
	std::vector<aPoint> m_aHome;
	std::vector<aRectangle> m_aObstacles;
	std::vector<aPoint> m_aVehicles;
	std::vector<aRectangle> m_aTasks;
	std::vector<std::vector<aPoint>> m_aPaths;
	std::vector<aPoint> m_aPath;
	aRectangle m_aRandArea;
	aRectangle m_aRectRefArea;
	
	double m_scalingFactor;
	double m_posShift;

	void CalcScalingFactor();
	void RemapAll();
	void RemapRectangle(aRectangle rectSrc, CRect &rcDst);
	void RemapPoint(aPoint aSrc, CPoint &ptDst);

	void CreateResources();
	void DestroyResources();
	void DrawData(CDC *pDC);
	void DrawObstacles(CDC *pDC);
	void DrawHomePositions(CDC *pDC);
	void DrawVehiclesPositions(CDC *pDC);
	void DrawTasks(CDC *pDC);
	void DrawPaths(CDC *pDC);

	CBrush m_brClientArea;
	CBrush m_brRandArea;
	CBrush m_brNavArea;
	CBrush m_brForbiddenArea;
	CBrush m_brHomePos;
	CBrush m_brVehiclePos;
	CBrush m_brTaskArea;
	CBrush m_brRed;
	CBrush m_brGreen;

	int m_missionStatus;

	std::vector<TaskStruct*> m_pTaskStructVector;
	void ClearTaskStruct();

	std::vector<CButton *> m_pCheckBoxVectorA;
	std::vector<CButton *> m_pCheckBoxVectorB;
	std::vector<CButton *> m_pRadioButtonVector;
	std::vector<CButton *> m_pPushButtonVector;
	std::vector<CEdit *> m_pEditVector;
	std::vector<CStatic *> m_pLabelVector;

	void CreateMasterEnableButton();
	void CreateEnableTaskButton();
	void CreateEnablePathButton();
	void CreateBwdButton();
	void CreateFwdButton();
	void CreateHomeButton(WPARAM id);
	void CreateVehicleButton(WPARAM id);
	void CreateTaskButton(WPARAM id);
	void CreatePathButton(WPARAM id, size_t items);
	void CreatePathRadioButton(WPARAM id);
	void CreateEdit(WPARAM ofsId);
	void CreateLabel(WPARAM ofsId, LPCTSTR label);
	void DestroyAllDynamicControls();
	int m_currentPathSelected;
	int m_currentWaypoint;

	bool m_bTest;
	void PrepareTestMission();
	void PrepareTestMissionNavArea(Mission &mission);
	void PrepareTestMissionForbiddenAreas(Mission &mission);
	void PrepareTestMissionHomeLocations(Mission &mission);
	void PrepareTestMissionVehicles(Mission &mission);
	void PrepareTestMissionTasks(Mission &mission);

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	afx_msg void OnMissionCheckboxEnableAll();
	afx_msg void OnMissionCheckboxEnableTasks();
	afx_msg void OnMissionCheckboxEnablePaths();
	afx_msg void OnMissionCheckboxClicked(UINT nID);
	afx_msg void OnPathRadioButtonClicked(UINT nID);
	afx_msg void OnBwdButtonClicked();
	afx_msg void OnFwdButtonClicked();
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnClose();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnDestroy();
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
};
