
// MlpGuiDlg.cpp : implementation file
//

#include "pch.h"
#include "framework.h"
#include "MlpGui.h"
#include "MlpGuiDlg.h"
#include "afxdialogex.h"
#include "WinMessages.h"
#include "Configurator.h"
#include "CoveragePlanner.h"
#include "AStarPlanner.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace geon;

#define	IDC_ENABLE_BUTTONS			1900
#define	IDC_PUSH_BUTTONS			1950
#define	IDC_HOME_BUTTONS_BASE		2000
#define	IDC_VEHICLE_BUTTONS_BASE	2100
#define	IDC_TASK_BUTTONS_BASE		2200
#define	IDC_PATH_BUTTONS_BASE		2300
#define	IDC_PATH_RADIOBUTTONS_BASE	2400
#define	IDC_EDITS_BASE				2500
#define	IDC_LABELS_BASE				2600

#define	ID_OFS_CURRENT_WAYPOINT_NR	0
#define	ID_OFS_CURRENT_WAYPOINT_LON	1
#define	ID_OFS_CURRENT_WAYPOINT_LAT	2
#define	ID_OFS_TASK_NAME			3
#define	ID_OFS_TASK_ID				4

#define	COL1_W				40
#define	COL2_W				160
#define	COL3_W				30
#define	COL4_W				160

#define	COL1_OFS_X			(COL1_W+COL2_W+COL3_W+COL4_W+GAP_X)
#define	COL2_OFS_X			(COL2_W+COL3_W+COL4_W+GAP_X)
#define	COL3_OFS_X			(COL3_W+COL4_W)
#define	COL4_OFS_X			(COL4_W)

#define	GAP_X				10
#define	GAP_Y				40
#define	OFS_Y				10

#define	WIDTH_CHECKBOXES	COL2_W
#define	WIDTH_LABELS		COL1_W
#define	WIDTH_PUSHBUTTONS	((COL2_W - GAP_X)/2)
#define	WIDTH_RADIOBUTTONS	(COL3_W)
#define	WIDTH_EDITS			(COL2_W)
#define	HEIGHT_CONTROLS		30

#define	MASTER_ENABLE_OFS_X	(COL2_OFS_X)
#define	BTN_BWD_X			(COL2_OFS_X)
#define	BTN_FWD_X			(BTN_BWD_X - WIDTH_PUSHBUTTONS - GAP_X)
#define	BTNS_BASE_Y			(OFS_Y + 3 * GAP_Y)

#define	EDITS_OFS_X			(COL2_OFS_X)
#define	EDITS_BASE_Y		(OFS_Y + 4 * GAP_Y)

#define	LABELS_OFS_X		(COL1_OFS_X)
#define	LABELS_BASE_Y		(EDITS_BASE_Y)

#define	RADIO_BUTTON_OFS_X	(COL3_OFS_X)
#define	CHECK_BUTTON_OFS_X	(COL4_OFS_X)

DWORD	WINAPI	lpfnMlpServerFun(CMlpGuiDlg* pDlg)
{
	CString s;
	s.Format(_T("Starting the MLP server (%d) ..."), pDlg->GetPort());
	pDlg->SetDlgItemText(IDC_STATUS, s);

	pDlg->GetServer()->serve();

	pDlg->SetDlgItemText(IDC_STATUS, _T("Server stopped"));

	return 0L;
}

void CMlpGuiDlg::OnMissionCheckboxClicked(UINT nID)
{
	RedrawWindow();
}

void CMlpGuiDlg::OnMissionCheckboxEnableAll()
{
	UINT checked = IsDlgButtonChecked(IDC_ENABLE_BUTTONS);
	for (size_t n = 0; n < m_pCheckBoxVectorA.size(); n++)
		m_pCheckBoxVectorA[n]->SetCheck(checked);
	CheckDlgButton(IDC_ENABLE_BUTTONS + 1, checked);
	CheckDlgButton(IDC_ENABLE_BUTTONS + 2, checked);
	RedrawWindow();
}

void CMlpGuiDlg::OnMissionCheckboxEnableTasks()
{
	UINT checked = IsDlgButtonChecked(IDC_ENABLE_BUTTONS + 1);
	for (UINT n = 0; n < m_rcTasks.size(); n++)
		CheckDlgButton(IDC_TASK_BUTTONS_BASE + n, checked);
	RedrawWindow();
}

void CMlpGuiDlg::OnMissionCheckboxEnablePaths()
{
	UINT checked = IsDlgButtonChecked(IDC_ENABLE_BUTTONS + 2);
	for (UINT n = 0; n < m_ptPaths.size(); n++)
		CheckDlgButton(IDC_PATH_BUTTONS_BASE + n, checked);
	RedrawWindow();
}

void CMlpGuiDlg::OnBwdButtonClicked()
{
	if (m_currentWaypoint > 0)
	{
		m_currentWaypoint--;
		m_pPushButtonVector[1]->EnableWindow(true);
		if (m_currentWaypoint == 0)
			m_pPushButtonVector[0]->EnableWindow(false);
		RedrawWindow();
	}
}

void CMlpGuiDlg::OnFwdButtonClicked()
{
	if (m_currentWaypoint < m_aPaths[m_currentPathSelected].size() - 1)
	{
		m_currentWaypoint++;
		m_pPushButtonVector[0]->EnableWindow(true);
		if (m_currentWaypoint == m_aPaths[m_currentPathSelected].size() - 1)
			m_pPushButtonVector[1]->EnableWindow(false);
		RedrawWindow();
	}
}

void CMlpGuiDlg::OnPathRadioButtonClicked(UINT nID)
{
	m_pPushButtonVector[0]->EnableWindow(false);
	m_pPushButtonVector[1]->EnableWindow(true);
	m_currentPathSelected = nID - IDC_PATH_RADIOBUTTONS_BASE;
	m_currentWaypoint = 0;
	string taskType = _TaskType_VALUES_TO_NAMES.at(m_pTaskStructVector[m_currentPathSelected]->taskType);
	std::wstring widestr = std::wstring(taskType.begin(), taskType.end());
	SetDlgItemText(IDC_EDITS_BASE + 3, widestr.c_str());
	SetDlgItemInt(IDC_EDITS_BASE + 4, m_pTaskStructVector[m_currentPathSelected]->id);
	RedrawWindow();
}

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CMlpGuiDlg dialog

void CMlpGuiDlg::ClearData()
{
	m_bRefAreaReceived = false;

	m_aObstacles.clear();
	m_aHome.clear();
	m_aVehicles.clear();
	m_aTasks.clear();
	m_aPaths.clear();
	m_aPath.clear();

	m_rcObstacles.clear();
	m_ptHome.clear();
	m_ptVehicles.clear();
	m_rcTasks.clear();
	m_ptPaths.clear();
	m_ptPath.clear();

	m_scalingFactor = 1.0;
	m_posShift = 0.0;

	m_currentPathSelected = -1;
	m_currentWaypoint = 0;

	m_missionStatus = -1;
}

CMlpGuiDlg::CMlpGuiDlg(LPTSTR lpCmdLine, CWnd* pParent /*=nullptr*/)
	: CDialog(IDD_MLPGUI_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDI_MLP);

	CreateResources();

	ClearData();

	Configurator::checkConfigurationFolders();

	CString sCmdLine = lpCmdLine;
	m_bTest = !(sCmdLine.CompareNoCase(_T("/test")));
}

void CMlpGuiDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CMlpGuiDlg, CDialog)
	ON_CONTROL_RANGE(BN_CLICKED, IDC_HOME_BUTTONS_BASE, IDC_HOME_BUTTONS_BASE + 99, OnMissionCheckboxClicked)
	ON_CONTROL_RANGE(BN_CLICKED, IDC_VEHICLE_BUTTONS_BASE, IDC_VEHICLE_BUTTONS_BASE + 99, OnMissionCheckboxClicked)
	ON_CONTROL_RANGE(BN_CLICKED, IDC_TASK_BUTTONS_BASE, IDC_TASK_BUTTONS_BASE + 99, OnMissionCheckboxClicked)
	ON_CONTROL_RANGE(BN_CLICKED, IDC_PATH_BUTTONS_BASE, IDC_PATH_BUTTONS_BASE + 99, OnMissionCheckboxClicked)
	ON_CONTROL_RANGE(BN_CLICKED, IDC_PATH_RADIOBUTTONS_BASE, IDC_PATH_RADIOBUTTONS_BASE + 99, OnPathRadioButtonClicked)
	ON_BN_CLICKED(IDC_ENABLE_BUTTONS, &CMlpGuiDlg::OnMissionCheckboxEnableAll)
	ON_BN_CLICKED(IDC_ENABLE_BUTTONS + 1, &CMlpGuiDlg::OnMissionCheckboxEnableTasks)
	ON_BN_CLICKED(IDC_ENABLE_BUTTONS + 2, &CMlpGuiDlg::OnMissionCheckboxEnablePaths)
	ON_BN_CLICKED(IDC_PUSH_BUTTONS, &CMlpGuiDlg::OnBwdButtonClicked)
	ON_BN_CLICKED(IDC_PUSH_BUTTONS + 1, &CMlpGuiDlg::OnFwdButtonClicked)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
//	ON_BN_CLICKED(IDC_BUTTON1, &CMlpGuiDlg::OnBnClickedButton1)
	ON_WM_CLOSE()
//	ON_WM_DESTROY()
	ON_WM_SIZE()
	ON_WM_DESTROY()
	ON_WM_CTLCOLOR()
END_MESSAGE_MAP()


// CMlpGuiDlg message handlers
void CMlpGuiDlg::CreateResources()
{
	m_brClientArea.CreateSolidBrush(RGB(83, 145, 227));
	m_brRandArea.CreateSolidBrush(RGB(235, 151, 75));
	m_brNavArea.CreateSolidBrush(RGB(0, 230, 57));
	m_brForbiddenArea.CreateSolidBrush(RGB(255, 0, 0));
	m_brHomePos.CreateSolidBrush(RGB(255, 0, 255));
	m_brVehiclePos.CreateSolidBrush(RGB(255, 255, 255));
	m_brTaskArea.CreateSolidBrush(RGB(0, 0, 255));

	m_brRed.CreateSolidBrush(RGB(255, 0, 0));
	m_brGreen.CreateSolidBrush(RGB(0, 255, 0));
}

void CMlpGuiDlg::DestroyResources()
{
	m_brClientArea.DeleteObject();
	m_brRandArea.DeleteObject();
	m_brNavArea.DeleteObject();
	m_brForbiddenArea.DeleteObject();
	m_brHomePos.DeleteObject();
	m_brVehiclePos.DeleteObject();
	m_brTaskArea.DeleteObject();
	
	m_brRed.DeleteObject();
	m_brGreen.DeleteObject();
}

BOOL CMlpGuiDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	ShowWindow(SW_MAXIMIZE);

	StartMlpServer();

	if (m_bTest)
		PrepareTestMission();

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CMlpGuiDlg::CreateMasterEnableButton()
{
	CString s;
	CRect rcDlg;
	GetClientRect(rcDlg);

	s = _T("Enable all");
	CButton *pButton = new CButton();
	if (pButton)
	{
		UINT style = WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX | WS_TABSTOP;
		CRect rc(
			rcDlg.Width() - MASTER_ENABLE_OFS_X, OFS_Y,
			rcDlg.Width() - MASTER_ENABLE_OFS_X + WIDTH_CHECKBOXES, OFS_Y + HEIGHT_CONTROLS
		);
		pButton->Create(s, style, rc, this, IDC_ENABLE_BUTTONS);
		pButton->SetCheck(BST_CHECKED);
		pButton->SetFont(this->GetFont());
		m_pCheckBoxVectorB.push_back(pButton);
	}
	else
		AfxMessageBox(_T("Error creating enable button"));
}

void CMlpGuiDlg::CreateEnableTaskButton()
{
	CString s;
	CRect rcDlg;
	GetClientRect(rcDlg);

	size_t nButtons = m_pCheckBoxVectorB.size();

	s = _T("Enable tasks");
	CButton *pButton = new CButton();
	if (pButton)
	{
		UINT style = WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX | WS_TABSTOP;
		CRect rc(
			rcDlg.Width() - MASTER_ENABLE_OFS_X, OFS_Y + nButtons * GAP_Y,
			rcDlg.Width() - MASTER_ENABLE_OFS_X + WIDTH_CHECKBOXES, OFS_Y + HEIGHT_CONTROLS + nButtons * GAP_Y
		);
		pButton->Create(s, style, rc, this, IDC_ENABLE_BUTTONS + 1);
		pButton->SetCheck(BST_CHECKED);
		pButton->SetFont(this->GetFont());
		m_pCheckBoxVectorB.push_back(pButton);
	}
	else
		AfxMessageBox(_T("Error creating enable button"));
}

void CMlpGuiDlg::CreateEnablePathButton()
{
	CString s;
	CRect rcDlg;
	GetClientRect(rcDlg);

	size_t nButtons = m_pCheckBoxVectorB.size();

	s = _T("Enable paths");
	CButton *pButton = new CButton();
	if (pButton)
	{
		UINT style = WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX | WS_TABSTOP;
		CRect rc(
			rcDlg.Width() - MASTER_ENABLE_OFS_X, OFS_Y + nButtons * GAP_Y,
			rcDlg.Width() - MASTER_ENABLE_OFS_X + WIDTH_CHECKBOXES, OFS_Y + HEIGHT_CONTROLS + nButtons * GAP_Y
		);
		pButton->Create(s, style, rc, this, IDC_ENABLE_BUTTONS + 2);
		pButton->SetCheck(BST_CHECKED);
		pButton->SetFont(this->GetFont());
		m_pCheckBoxVectorB.push_back(pButton);
	}
	else
		AfxMessageBox(_T("Error creating enable button"));
}

void CMlpGuiDlg::CreateBwdButton()
{
	CRect rcDlg;
	GetClientRect(rcDlg);

	CButton *pButton = new CButton();
	if (pButton)
	{
		UINT style = WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_TABSTOP;
		CRect rc(
			rcDlg.Width() - BTN_BWD_X, BTNS_BASE_Y,
			rcDlg.Width() - BTN_BWD_X + WIDTH_PUSHBUTTONS, BTNS_BASE_Y + HEIGHT_CONTROLS
		);
		pButton->Create(_T("<"), style, rc, this, IDC_PUSH_BUTTONS);
		pButton->EnableWindow(false);
		m_pPushButtonVector.push_back(pButton);
	}
	else
		AfxMessageBox(_T("Error creating bwd button"));
}

void CMlpGuiDlg::CreateFwdButton()
{
	CRect rcDlg;
	GetClientRect(rcDlg);

	CButton *pButton = new CButton();
	if (pButton)
	{
		UINT style = WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_TABSTOP;
		CRect rc(
			rcDlg.Width() - BTN_FWD_X, BTNS_BASE_Y,
			rcDlg.Width() - BTN_FWD_X + WIDTH_PUSHBUTTONS, BTNS_BASE_Y + HEIGHT_CONTROLS
		);
		pButton->Create(_T(">"), style, rc, this, IDC_PUSH_BUTTONS + 1);
		pButton->EnableWindow(false);
		m_pPushButtonVector.push_back(pButton);
	}
	else
		AfxMessageBox(_T("Error creating fwd button"));
}

void CMlpGuiDlg::CreateEdit(WPARAM ofsId)
{
	CRect rcDlg;
	GetClientRect(rcDlg);

	CEdit *pEdit = new CEdit();
	if (pEdit)
	{
		UINT style = WS_CHILD | WS_VISIBLE | ES_CENTER | ES_READONLY | WS_TABSTOP | WS_BORDER;
		CRect rc(
			rcDlg.Width() - EDITS_OFS_X, EDITS_BASE_Y + ofsId * GAP_Y,
			rcDlg.Width() - EDITS_OFS_X + WIDTH_EDITS, EDITS_BASE_Y + ofsId * GAP_Y + HEIGHT_CONTROLS
		);
		pEdit->Create(style, rc, this, IDC_EDITS_BASE + ofsId);
		pEdit->SetFont(this->GetFont());
		m_pEditVector.push_back(pEdit);
	}
	else
		AfxMessageBox(_T("Error creating edit ctrl"));
}

void CMlpGuiDlg::CreateLabel(WPARAM ofsId, LPCTSTR label)
{
	CRect rcDlg;
	GetClientRect(rcDlg);

	CStatic *pLabel = new CStatic();
	if (pLabel)
	{
		UINT style = WS_CHILD | WS_VISIBLE | SS_LEFT | WS_TABSTOP;
		CRect rc(
			rcDlg.Width() - LABELS_OFS_X, LABELS_BASE_Y + ofsId * GAP_Y,
			rcDlg.Width() - LABELS_OFS_X + WIDTH_LABELS, LABELS_BASE_Y + ofsId * GAP_Y + HEIGHT_CONTROLS
		);
		pLabel->Create(label, style, rc, this, IDC_LABELS_BASE + ofsId);
		pLabel->SetFont(this->GetFont());
		m_pLabelVector.push_back(pLabel);
	}
	else
		AfxMessageBox(_T("Error creating static ctrl"));
}

void CMlpGuiDlg::CreateHomeButton(WPARAM id)
{
	CString s;
	CRect rcDlg;
	GetClientRect(rcDlg);

	size_t nButtons = m_pCheckBoxVectorA.size();

	s.Format(_T("Home %d"), id);
	CButton *pButton = new CButton();
	if (pButton)
	{
		UINT style = WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX | WS_TABSTOP;
		CRect rc(
			rcDlg.Width() - CHECK_BUTTON_OFS_X, OFS_Y + nButtons * GAP_Y,
			rcDlg.Width() - CHECK_BUTTON_OFS_X + WIDTH_CHECKBOXES, OFS_Y + HEIGHT_CONTROLS + nButtons * GAP_Y
		);
		pButton->Create(s, style, rc, this, IDC_HOME_BUTTONS_BASE + id);
		pButton->SetCheck(BST_CHECKED);
		m_pCheckBoxVectorA.push_back(pButton);
	}
	else
		AfxMessageBox(_T("Error creating home button"));
}

void CMlpGuiDlg::CreateVehicleButton(WPARAM id)
{
	CString s;
	CRect rcDlg;
	GetClientRect(rcDlg);

	size_t nButtons = m_pCheckBoxVectorA.size();

	s.Format(_T("Vehicle %d"), id);
	CButton *pButton = new CButton();
	if (pButton)
	{
		UINT style = WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX | WS_TABSTOP;
		CRect rc(
			rcDlg.Width() - CHECK_BUTTON_OFS_X, OFS_Y + nButtons * GAP_Y,
			rcDlg.Width() - CHECK_BUTTON_OFS_X + WIDTH_CHECKBOXES, OFS_Y + HEIGHT_CONTROLS + nButtons * GAP_Y
		);
		pButton->Create(s, style, rc, this, IDC_VEHICLE_BUTTONS_BASE + id);
		pButton->SetCheck(BST_CHECKED);
		m_pCheckBoxVectorA.push_back(pButton);
	}
	else
		AfxMessageBox(_T("Error creating vehicle button"));
}

void CMlpGuiDlg::CreateTaskButton(WPARAM id)
{
	CString s;
	CRect rcDlg;
	GetClientRect(rcDlg);

	size_t nButtons = m_pCheckBoxVectorA.size();

	s.Format(_T("Task %d"), id);
	CButton *pButton = new CButton();
	if (pButton)
	{
		UINT style = WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX | WS_TABSTOP;
		CRect rc(
			rcDlg.Width() - CHECK_BUTTON_OFS_X, OFS_Y + nButtons * GAP_Y,
			rcDlg.Width() - CHECK_BUTTON_OFS_X + WIDTH_CHECKBOXES, OFS_Y + HEIGHT_CONTROLS + nButtons * GAP_Y
		);
		pButton->Create(s, style, rc, this, IDC_TASK_BUTTONS_BASE + id);
		pButton->SetCheck(BST_CHECKED);
		m_pCheckBoxVectorA.push_back(pButton);
	}
	else
		AfxMessageBox(_T("Error creating task button"));
}

void CMlpGuiDlg::CreatePathButton(WPARAM id, size_t items)
{
	CString s;
	CRect rcDlg;
	GetClientRect(rcDlg);

	size_t nButtons = m_pCheckBoxVectorA.size();

	s.Format(_T("Path %d [%d]"), id, items);
	CButton *pButton = new CButton();
	if (pButton)
	{
		UINT style = WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX | WS_TABSTOP;
		CRect rc(
			rcDlg.Width() - CHECK_BUTTON_OFS_X, OFS_Y + nButtons * GAP_Y,
			rcDlg.Width() - CHECK_BUTTON_OFS_X + WIDTH_CHECKBOXES, OFS_Y + HEIGHT_CONTROLS + nButtons * GAP_Y
		);
		pButton->Create(s, style, rc, this, IDC_PATH_BUTTONS_BASE + id);
		pButton->SetCheck(BST_CHECKED);
		m_pCheckBoxVectorA.push_back(pButton);
	}
	else
		AfxMessageBox(_T("Error creating path button"));
}

void CMlpGuiDlg::CreatePathRadioButton(WPARAM id)
{
	CRect rcDlg;
	GetClientRect(rcDlg);

	size_t nButtons = m_pCheckBoxVectorA.size() - 1;

	CButton *pButton = new CButton();
	if (pButton)
	{
		UINT style = WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | BS_VCENTER | WS_TABSTOP;
		if (0 == id)
			style |= WS_GROUP;
		CRect rc(
			rcDlg.Width() - RADIO_BUTTON_OFS_X, OFS_Y + nButtons * GAP_Y,
			rcDlg.Width() - RADIO_BUTTON_OFS_X + COL3_W, OFS_Y + HEIGHT_CONTROLS + nButtons * GAP_Y
		);
		pButton->Create(_T(""), style, rc, this, IDC_PATH_RADIOBUTTONS_BASE + id);
		m_pRadioButtonVector.push_back(pButton);
	}
	else
		AfxMessageBox(_T("Error creating path radio button"));
}

void CMlpGuiDlg::DestroyAllDynamicControls()
{
	for (size_t n = 0; n < m_pCheckBoxVectorA.size(); n++)
	{
		delete m_pCheckBoxVectorA[n];
	}
	for (size_t n = 0; n < m_pCheckBoxVectorB.size(); n++)
	{
		delete m_pCheckBoxVectorB[n];
	}
	for (size_t n = 0; n < m_pRadioButtonVector.size(); n++)
	{
		delete m_pRadioButtonVector[n];
	}
	for (size_t n = 0; n < m_pPushButtonVector.size(); n++)
	{
		delete m_pPushButtonVector[n];
	}
	for (size_t n = 0; n < m_pEditVector.size(); n++)
	{
		delete m_pEditVector[n];
	}
	for (size_t n = 0; n < m_pLabelVector.size(); n++)
	{
		delete m_pLabelVector[n];
	}
	m_pCheckBoxVectorA.clear();
	m_pCheckBoxVectorB.clear();
	m_pRadioButtonVector.clear();
	m_pPushButtonVector.clear();
	m_pEditVector.clear();
	m_pLabelVector.clear();
}

void CMlpGuiDlg::RemapAll()
{
	CRect rc;
	CPoint pt;

	RemapRectangle(m_aRandArea, m_rcRandArea);
	RemapRectangle(m_aRectRefArea, m_rcNavArea);
	
	m_ptHome.clear();
	for (size_t n = 0; n < m_aHome.size(); n++)
	{
		RemapPoint(m_aHome[n], pt);
		m_ptHome.push_back(pt);
	}
	
	m_rcObstacles.clear();
	for (size_t n = 0; n < m_aObstacles.size(); n++)
	{
		RemapRectangle(m_aObstacles[n], rc);
		m_rcObstacles.push_back(rc);
	}
	
	m_ptVehicles.clear();
	for (size_t n = 0; n < m_aVehicles.size(); n++)
	{
		RemapPoint(m_aVehicles[n], pt);
		m_ptVehicles.push_back(pt);
	}
	
	m_rcTasks.clear();
	for (size_t n = 0; n < m_aTasks.size(); n++)
	{
		RemapRectangle(m_aTasks[n], rc);
		m_rcTasks.push_back(rc);
	}

	m_ptPaths.clear();
	for (size_t allp = 0; allp < m_aPaths.size(); allp++)
	{
		std::vector<aPoint> path = m_aPaths[allp];

		m_ptPath.clear();
		for (size_t np = 0; np < path.size(); np++)
		{
			RemapPoint(path[np], pt);
			m_ptPath.push_back(pt);
		}
		m_ptPaths.push_back(m_ptPath);
	}
}

void CMlpGuiDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CMlpGuiDlg::OnPaint()
{
	CPaintDC dc(this); // device context for painting

	if (IsIconic())
	{
		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		//CDialogEx::OnPaint();
		DrawData(&dc);
	}
}

void CMlpGuiDlg::DrawObstacles(CDC *pDC)
{
	CString s;
	CSize sz;
	int x, y;
	int oldBkMode;
	for (size_t na = 0; na < m_rcObstacles.size(); na++)
	{
		pDC->FillRect(m_rcObstacles[na], &m_brForbiddenArea);
		s.Format(_T("Obstacle %d"), na);
		sz = pDC->GetTextExtent(s);
		x = m_rcObstacles[na].left + (m_rcObstacles[na].Width() - sz.cx) / 2;
		y = m_rcObstacles[na].top + (m_rcObstacles[na].Height() - sz.cy) / 2;
		oldBkMode = pDC->SetBkMode(TRANSPARENT);
		pDC->TextOutW(x, y, s);
		pDC->SetBkMode(oldBkMode);
	}
}

void CMlpGuiDlg::DrawHomePositions(CDC *pDC)
{
	LOGBRUSH logBrush;
	CPen drawPen, *pOldPen;

	m_brHomePos.GetLogBrush(&logBrush);
	drawPen.CreatePen(PS_SOLID, 3, &logBrush, 0, NULL);
	pOldPen = pDC->SelectObject(&drawPen);
	for (size_t nh = 0; nh < m_ptHome.size(); nh++)
	{
		if (BST_CHECKED == IsDlgButtonChecked(IDC_HOME_BUTTONS_BASE + nh))
		{
			pDC->MoveTo(m_ptHome[nh].x - 5, m_ptHome[nh].y);
			pDC->LineTo(m_ptHome[nh].x + 5, m_ptHome[nh].y);
			pDC->MoveTo(m_ptHome[nh].x, m_ptHome[nh].y - 5);
			pDC->LineTo(m_ptHome[nh].x, m_ptHome[nh].y + 5);
		}
	}
	pDC->SelectObject(pOldPen);
	drawPen.DeleteObject();
}

void CMlpGuiDlg::DrawVehiclesPositions(CDC *pDC)
{
	CPen drawPen, *pOldPen;
	LOGBRUSH logBrush;

	m_brVehiclePos.GetLogBrush(&logBrush);
	drawPen.CreatePen(PS_SOLID, 2, &logBrush, 0, NULL);
	pOldPen = pDC->SelectObject(&drawPen);
	for (size_t nv = 0; nv < m_ptVehicles.size(); nv++)
	{
		if (BST_CHECKED == IsDlgButtonChecked(IDC_VEHICLE_BUTTONS_BASE + nv))
		{
			pDC->Ellipse(
				m_ptVehicles[nv].x - 5, m_ptVehicles[nv].y - 5,
				m_ptVehicles[nv].x + 5, m_ptVehicles[nv].y + 5
			);
			/*
			pDC->MoveTo(m_ptVehicles[nv].x - 5, m_ptVehicles[nv].y);
			pDC->LineTo(m_ptVehicles[nv].x + 5, m_ptVehicles[nv].y);
			pDC->MoveTo(m_ptVehicles[nv].x, m_ptVehicles[nv].y - 5);
			pDC->LineTo(m_ptVehicles[nv].x, m_ptVehicles[nv].y + 5);
			*/
		}
	}
	pDC->SelectObject(pOldPen);
	drawPen.DeleteObject();
}

void CMlpGuiDlg::DrawTasks(CDC *pDC)
{
	CString s;
	CSize sz;
	int x, y;
	int oldBkMode;

	for (size_t nt = 0; nt < m_rcTasks.size(); nt++)
	{
		if (BST_CHECKED == IsDlgButtonChecked(IDC_TASK_BUTTONS_BASE + nt))
		{
			pDC->FrameRect(m_rcTasks[nt], &m_brTaskArea);
			s.Format(_T("Task %d"), nt);
			sz = pDC->GetTextExtent(s);
			x = m_rcTasks[nt].right + 5;
			y = m_rcTasks[nt].top + (m_rcTasks[nt].Height() - sz.cy) / 2;
			oldBkMode = pDC->SetBkMode(TRANSPARENT);
			pDC->TextOutW(x, y, s);
			pDC->SetBkMode(oldBkMode);
		}
	}
}

void CMlpGuiDlg::DrawPaths(CDC *pDC)
{
	CString s;
	CPoint waypoint;
	CPen drawPen, *pOldPen;

	for (size_t allp = 0; allp < m_ptPaths.size(); allp++)
	{
		if (BST_CHECKED == IsDlgButtonChecked(IDC_PATH_BUTTONS_BASE + allp))
		{
			BYTE r = (BYTE)(allp * 20);
			BYTE g = (BYTE)(allp * 16);
			BYTE b = (BYTE)(allp * 8);
			drawPen.CreatePen(PS_SOLID, 1, RGB(r, g, b));
			pOldPen = pDC->SelectObject(&drawPen);

			std::vector<CPoint> path = m_ptPaths[allp];
			if (path.size() >= 2)
			{
				waypoint = path[0];
				pDC->Ellipse(
					waypoint.x - 10, waypoint.y - 10,
					waypoint.x + 10, waypoint.y + 10
				);

				pDC->MoveTo(waypoint);
				for (size_t np = 1; np < path.size(); np++)
				{
					waypoint = path[np];
					pDC->LineTo(waypoint);
				}

				waypoint = path[path.size() - 1];
				pDC->Ellipse(
					waypoint.x - 10, waypoint.y - 10,
					waypoint.x + 10, waypoint.y + 10
				);
			}

			if (allp == m_currentPathSelected)
			{
				pointVec apath = m_aPaths[allp];
				SetDlgItemInt(IDC_EDITS_BASE, m_currentWaypoint);
				s.Format(_T("%.5f"), apath[m_currentWaypoint].x());
				SetDlgItemText(IDC_EDITS_BASE + 1, s);
				s.Format(_T("%.5f"), apath[m_currentWaypoint].y());
				SetDlgItemText(IDC_EDITS_BASE + 2, s);
				waypoint = path[m_currentWaypoint];
				pDC->MoveTo(waypoint.x - 15, waypoint.y - 15);
				pDC->LineTo(waypoint.x + 15, waypoint.y + 15);
				pDC->MoveTo(waypoint.x - 15, waypoint.y + 15);
				pDC->LineTo(waypoint.x + 15, waypoint.y - 15);
			}

			pDC->SelectObject(pOldPen);
			drawPen.DeleteObject();
		}
	}
}

void CMlpGuiDlg::DrawData(CDC *pDC)
{
	//pDC->FrameRect(m_rcClient, &m_brClientArea);
	if (m_bRefAreaReceived)
	{
		//pDC->FrameRect(m_rcRandArea, &m_brRandArea);
		pDC->FrameRect(m_rcNavArea, &m_brNavArea);
		DrawObstacles(pDC);
		DrawHomePositions(pDC);
		DrawVehiclesPositions(pDC);
		DrawTasks(pDC);
		DrawPaths(pDC);
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CMlpGuiDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CMlpGuiDlg::StartMlpServer()
{
	DWORD	threadID;

	hostParamsMap hostMap;
	Configurator cfg("./MLP.json");
	if (cfg.loadHostsConfiguration(hostMap))
	{
		try
		{
			m_port = hostMap["MLP"].port;
		}
		catch (const std::exception&)
		{
			m_port = MLP_HOST_PORT;
		}
	}
	else
	{
		MessageBox(_T("Unable to read configuration file"), _T("Error"), MB_ICONERROR | MB_OK);
	}

	WSADATA wsaData = {};
	WORD wVersionRequested = MAKEWORD(2, 2);
	int err = WSAStartup(wVersionRequested, &wsaData);
	boost::shared_ptr<TProtocolFactory> protocolFactory(new TBinaryProtocolFactory());
	boost::shared_ptr<PlannerServiceHandler> handler(new PlannerServiceHandler(this));
	boost::shared_ptr<TProcessor> processor(new PlannerServiceProcessor(handler));
	boost::shared_ptr<TServerTransport> serverTransport(new TServerSocket(m_port));
	boost::shared_ptr<TTransportFactory> transportFactory(new TBufferedTransportFactory());

	m_pServer = new TSimpleServer(processor,
		serverTransport,
		transportFactory,
		protocolFactory);

	m_hMlpServerThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)lpfnMlpServerFun, this, 0, &threadID);
}

void CMlpGuiDlg::OnClose()
{
	if (IDYES == MessageBox(_T("Are you sure you want to exit ?"), _T("Warning"), MB_ICONQUESTION | MB_YESNO | MB_DEFBUTTON2))
	{
		m_pServer->stop();

		DestroyAllDynamicControls();
		ClearTaskStruct();

		CDialog::OnClose();
	}
}

void CMlpGuiDlg::CalcScalingFactor()
{
	double fx = fabs(m_rcClient.Width() / m_aRectRefArea.width());
	double fy = fabs(m_rcClient.Height() / m_aRectRefArea.height());
	m_scalingFactor = min(fx, fy);
	m_posShift = 10.0 / m_scalingFactor;
}

void CMlpGuiDlg::RemapRectangle(aRectangle rectSrc, CRect &rcDst)
{
	rcDst.left = m_rcClient.left + (int)((rectSrc.left() - m_aRectRefArea.left())*m_scalingFactor);
	rcDst.bottom = m_rcClient.top + m_rcClient.Height() - (int)((rectSrc.bottom() - m_aRectRefArea.bottom())*m_scalingFactor);
	rcDst.right = m_rcClient.left + (int)((rectSrc.right() - m_aRectRefArea.left())*m_scalingFactor);
	rcDst.top = m_rcClient.top - (int)((rectSrc.top() - m_aRectRefArea.top())*m_scalingFactor);
	/*
	TRACE(_T("S(%f,%f,%f,%f) D(%d,%d,%d,%d) SF(%f)"),
		rectSrc.left(), rectSrc.bottom(), rectSrc.right(), rectSrc.top(),
		rcDst.left, rcDst.bottom, rcDst.right, rcDst.top,
		m_scalingFactor
	);
	*/
}

void CMlpGuiDlg::RemapPoint(aPoint aSrc, CPoint &ptDst)
{
	ptDst.x = m_rcClient.left + (int)((aSrc.x() - m_aRectRefArea.left())*m_scalingFactor);
	ptDst.y = m_rcClient.top + m_rcClient.Height() - (int)((aSrc.y() - m_aRectRefArea.bottom())*m_scalingFactor);
}

void CMlpGuiDlg::ClearTaskStruct()
{
	for (size_t n = 0; n < m_pTaskStructVector.size(); n++)
	{
		delete m_pTaskStructVector[n];
	}
	m_pTaskStructVector.clear();
}

BOOL CMlpGuiDlg::PreTranslateMessage(MSG* pMsg)
{
	Region *pRegion;
	Position *pPosition;
	aRectangle aRect;
	aRectangle *paRect;
	aPoint aPt;
	aPoint *paPt;
	CRect rc;
	CPoint pt;
	CString sz;
	UINT id, num;
	wchar_t* s;
	TaskStruct *pTaskStruct;

	switch (pMsg->message)
	{
	case WM_MLP_STATUS:
		switch (pMsg->wParam)
		{
		case WP_MLP_START:
			SetDlgItemText(IDC_STATUS, _T(""));
			ClearData();
			ClearTaskStruct();
			DestroyAllDynamicControls();
			CreateMasterEnableButton();
			CreateEnableTaskButton();
			CreateEnablePathButton();
			CreateBwdButton();
			CreateFwdButton();
			CreateLabel(ID_OFS_CURRENT_WAYPOINT_NR, _T("WP"));
			CreateEdit(ID_OFS_CURRENT_WAYPOINT_NR);
			CreateLabel(ID_OFS_CURRENT_WAYPOINT_LON, _T("LON"));
			CreateEdit(ID_OFS_CURRENT_WAYPOINT_LON);
			CreateLabel(ID_OFS_CURRENT_WAYPOINT_LAT, _T("LAT"));
			CreateEdit(ID_OFS_CURRENT_WAYPOINT_LAT);
			CreateLabel(ID_OFS_TASK_NAME, _T("N"));
			CreateEdit(ID_OFS_TASK_NAME);
			CreateLabel(ID_OFS_TASK_ID, _T("ID"));
			CreateEdit(ID_OFS_TASK_ID);
			break;
		}
		return TRUE;
	case WM_MLP_INFO:
		m_missionStatus = pMsg->wParam;
		s = (wchar_t*)pMsg->lParam;
		SetDlgItemText(IDC_STATUS, s);
		delete s;
		return TRUE;
	case WM_MLP_MISSION_DURATION:
		sz.Format(_T("Elapsed time : %.4f seconds"), (double)((int)pMsg->lParam / 10000.0));
		SetDlgItemText(IDC_MISSION_DURATION, sz);
		return TRUE;
	case WM_MLP_RAND_AREA:
		paRect = (aRectangle*)pMsg->lParam;
		delete paRect;
		return TRUE;
	case WM_MLP_NAV_AREA:
		pRegion = (Region*)pMsg->lParam;
		m_aRectRefArea = aRectangle(
			pRegion->area[0].longitude,
			pRegion->area[0].latitude,
			pRegion->area[2].longitude - pRegion->area[0].longitude,
			pRegion->area[2].latitude - pRegion->area[0].latitude
		);
		delete pRegion;
		m_bRefAreaReceived = true;
		CalcScalingFactor();
		RemapAll();
		RedrawWindow();
		return TRUE;
	case WM_MLP_OBS_AREA:
		pRegion = (Region*)pMsg->lParam;
		aRect = aRectangle(
			pRegion->area[0].longitude,
			pRegion->area[0].latitude,
			pRegion->area[2].longitude - pRegion->area[0].longitude,
			pRegion->area[2].latitude - pRegion->area[0].latitude
		);
		delete pRegion;
		m_aObstacles.push_back(aRect);
		if (m_bRefAreaReceived)
		{
			RemapAll();
			RedrawWindow();
		}
		return TRUE;
	case WM_MLP_HOME_POS:
		CreateHomeButton(pMsg->wParam);
		pPosition = (Position*)pMsg->lParam;
		aPt = aPoint(
			pPosition->longitude,
			pPosition->latitude
		);
		delete pPosition;
		m_aHome.push_back(aPt);
		if (m_bRefAreaReceived)
		{
			RemapAll();
			RedrawWindow();
		}
		return TRUE;
	case WM_MLP_VEHICLE_POS:
		CreateVehicleButton(pMsg->wParam);
		pPosition = (Position*)pMsg->lParam;
		aPt = aPoint(
			pPosition->longitude,
			pPosition->latitude
		);
		delete pPosition;
		m_aVehicles.push_back(aPt);
		if (m_bRefAreaReceived)
		{
			RemapAll();
			RedrawWindow();
		}
		return TRUE;
	case WM_MLP_TASK_AREA:
		CreateTaskButton(pMsg->wParam);
		pRegion = (Region*)pMsg->lParam;
		if (pRegion->area.size() == 1)
		{
			pPosition = &pRegion->area[0];
			aRect = aRectangle(
				pPosition->longitude - m_posShift,
				pPosition->latitude - m_posShift,
				2.0 * m_posShift,
				2.0 * m_posShift
			);
		}
		else
		{
			aRect = aRectangle(
				pRegion->area[0].longitude,
				pRegion->area[0].latitude,
				pRegion->area[1].longitude - pRegion->area[0].longitude,
				pRegion->area[1].latitude - pRegion->area[0].latitude
			);
		}
		delete pRegion;
		m_aTasks.push_back(aRect);
		if (m_bRefAreaReceived)
		{
			RemapAll();
			RedrawWindow();
		}
		return TRUE;
	case WM_MLP_PATH_BEGIN:
		pTaskStruct = (TaskStruct*)pMsg->lParam;
		m_pTaskStructVector.push_back(pTaskStruct);
		m_aPath.clear();
		return TRUE;
	case WM_MLP_PATH_END:
		CreatePathButton(pMsg->wParam, m_aPath.size());
		CreatePathRadioButton(pMsg->wParam);
		m_aPaths.push_back(m_aPath);
		if (m_bRefAreaReceived)
		{
			RemapAll();
			RedrawWindow();
		}
		return TRUE;
	case WM_MLP_WAYPOINT:
		id = (pMsg->wParam >> 16) & 0xFFFF;
		num = pMsg->wParam & 0xFFFF;
		paPt = (aPoint*)pMsg->lParam;
		m_aPath.push_back(*paPt);
		delete paPt;
		return TRUE;
	}
	return FALSE;
}


void CMlpGuiDlg::OnSize(UINT nType, int cx, int cy)
{
	CDialog::OnSize(nType, cx, cy);

	GetClientRect(m_rcClient);
	m_rcClient.DeflateRect(10, 80, 10, 50);
	if (m_bRefAreaReceived)
	{
		CalcScalingFactor();
		RemapAll();
		RedrawWindow();
	}
}

void CMlpGuiDlg::OnDestroy()
{
	CDialog::OnDestroy();
	DestroyResources();
}

HBRUSH CMlpGuiDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
	HBRUSH hbr = CDialog::OnCtlColor(pDC, pWnd, nCtlColor);
	
	int nID = pWnd->GetDlgCtrlID();

	if ((nID == IDC_PUSH_BUTTONS) || (nID == IDC_PUSH_BUTTONS + 1))
	{
		int index = nID - IDC_PUSH_BUTTONS;
		if (m_pPushButtonVector.size() > index)
		{
			if (m_pPushButtonVector[index]->IsWindowEnabled())
			{
				pDC->SetBkColor(RGB(0, 255, 0));
				pDC->SetBkMode(TRANSPARENT);
				hbr = m_brGreen;
			}
			else
			{
				pDC->SetBkColor(RGB(255, 0, 0));
				pDC->SetBkMode(TRANSPARENT);
				hbr = m_brRed;
			}
		}
	}
	if (pWnd->GetDlgCtrlID() == IDC_STATUS)
	{
		if (WP_MLP_INFO_SUCCESS == m_missionStatus)
		{
			pDC->SetBkColor(RGB(0, 255, 0));
			hbr = m_brGreen;
		}
		else if (WP_MLP_INFO_FAIL == m_missionStatus)
		{
			pDC->SetBkColor(RGB(255, 0, 0));
			hbr = m_brRed;
		}
	}

	return hbr;
}

#define	TM_ID				1234

#define	TM_NAV_AREA_X0		(-9.24218)
#define	TM_NAV_AREA_Y0		(38.7363)
#define	TM_NAV_AREA_X1		(-9.23507)
#define	TM_NAV_AREA_Y1		(38.7405)
#define	TM_ALTITUDE			30.0

#define	RESCALE_LON(x)		(TM_NAV_AREA_X0 + (x)*(TM_NAV_AREA_X1-TM_NAV_AREA_X0)/1920.0)
#define	RESCALE_LAT(l)		(TM_NAV_AREA_Y0 + (l)*(TM_NAV_AREA_Y1-TM_NAV_AREA_Y0)/1080.0)

void CMlpGuiDlg::PrepareTestMissionNavArea(Mission &mission)
{
	Position pos;
	Region navArea;

	pos.__set_longitude(TM_NAV_AREA_X0);
	pos.__set_latitude(TM_NAV_AREA_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	navArea.area.push_back(pos);
	pos.__set_longitude(TM_NAV_AREA_X0);
	pos.__set_latitude(TM_NAV_AREA_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	navArea.area.push_back(pos);
	pos.__set_longitude(TM_NAV_AREA_X1);
	pos.__set_latitude(TM_NAV_AREA_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	navArea.area.push_back(pos);
	pos.__set_longitude(TM_NAV_AREA_X1);
	pos.__set_latitude(TM_NAV_AREA_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	navArea.area.push_back(pos);
	pos.__set_longitude(TM_NAV_AREA_X0);
	pos.__set_latitude(TM_NAV_AREA_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	navArea.area.push_back(pos);

	mission.__set_navigationArea(navArea);
}

/*
#define	TM_OBS_0_X0			RESCALE_LON(400.0)
#define	TM_OBS_0_Y0			RESCALE_LAT(400.0)
#define	TM_OBS_0_X1			RESCALE_LON(800.0)
#define	TM_OBS_0_Y1			RESCALE_LAT(800.0)
*/
#define	TM_OBS_1_X0			RESCALE_LON(10.0)
#define	TM_OBS_1_Y0			RESCALE_LAT(500.0)
#define	TM_OBS_1_X1			RESCALE_LON(490.0)
#define	TM_OBS_1_Y1			RESCALE_LAT(700.0)

#define	TM_OBS_2_X0			RESCALE_LON(500.0)
#define	TM_OBS_2_Y0			RESCALE_LAT(610.0)
#define	TM_OBS_2_X1			RESCALE_LON(700.0)
#define	TM_OBS_2_Y1			RESCALE_LAT(1050.0)

#define	TM_OBS_3_X0			RESCALE_LON(710.0)
#define	TM_OBS_3_Y0			RESCALE_LAT(500.0)
#define	TM_OBS_3_X1			RESCALE_LON(1200.0)
#define	TM_OBS_3_Y1			RESCALE_LAT(700.0)

#define	TM_OBS_4_X0			RESCALE_LON(500.0)
#define	TM_OBS_4_Y0			RESCALE_LAT(100.0)
#define	TM_OBS_4_X1			RESCALE_LON(700.0)
#define	TM_OBS_4_Y1			RESCALE_LAT(590.0)

#define	TM_OBS_5_X0			RESCALE_LON(1400.0)
#define	TM_OBS_5_Y0			RESCALE_LAT(500.0)
#define	TM_OBS_5_X1			RESCALE_LON(1800.0)
#define	TM_OBS_5_Y1			RESCALE_LAT(700.0)

void CMlpGuiDlg::PrepareTestMissionForbiddenAreas(Mission &mission)
{
	Position pos;
	Region obsArea;
	std::vector<Region> obstacles;

	/*
	pos.__set_longitude(TM_OBS_0_X0);
	pos.__set_latitude(TM_OBS_0_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_0_X0);
	pos.__set_latitude(TM_OBS_0_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_0_X1);
	pos.__set_latitude(TM_OBS_0_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_0_X1);
	pos.__set_latitude(TM_OBS_0_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_0_X0);
	pos.__set_latitude(TM_OBS_0_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	obstacles.push_back(obsArea);
	obsArea.area.clear();
	*/

	pos.__set_longitude(TM_OBS_1_X0);
	pos.__set_latitude(TM_OBS_1_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_1_X0);
	pos.__set_latitude(TM_OBS_1_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_1_X1);
	pos.__set_latitude(TM_OBS_1_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_1_X1);
	pos.__set_latitude(TM_OBS_1_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_1_X0);
	pos.__set_latitude(TM_OBS_1_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	obstacles.push_back(obsArea);
	obsArea.area.clear();

	pos.__set_longitude(TM_OBS_2_X0);
	pos.__set_latitude(TM_OBS_2_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_2_X0);
	pos.__set_latitude(TM_OBS_2_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_2_X1);
	pos.__set_latitude(TM_OBS_2_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_2_X1);
	pos.__set_latitude(TM_OBS_2_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_2_X0);
	pos.__set_latitude(TM_OBS_2_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	obstacles.push_back(obsArea);
	obsArea.area.clear();

	pos.__set_longitude(TM_OBS_3_X0);
	pos.__set_latitude(TM_OBS_3_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_3_X0);
	pos.__set_latitude(TM_OBS_3_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_3_X1);
	pos.__set_latitude(TM_OBS_3_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_3_X1);
	pos.__set_latitude(TM_OBS_3_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_3_X0);
	pos.__set_latitude(TM_OBS_3_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	obstacles.push_back(obsArea);
	obsArea.area.clear();

	pos.__set_longitude(TM_OBS_4_X0);
	pos.__set_latitude(TM_OBS_4_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_4_X0);
	pos.__set_latitude(TM_OBS_4_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_4_X1);
	pos.__set_latitude(TM_OBS_4_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_4_X1);
	pos.__set_latitude(TM_OBS_4_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_4_X0);
	pos.__set_latitude(TM_OBS_4_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	obstacles.push_back(obsArea);
	obsArea.area.clear();

	pos.__set_longitude(TM_OBS_5_X0);
	pos.__set_latitude(TM_OBS_5_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_5_X0);
	pos.__set_latitude(TM_OBS_5_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_5_X1);
	pos.__set_latitude(TM_OBS_5_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_5_X1);
	pos.__set_latitude(TM_OBS_5_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	pos.__set_longitude(TM_OBS_5_X0);
	pos.__set_latitude(TM_OBS_5_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	obsArea.area.push_back(pos);
	obstacles.push_back(obsArea);
	obsArea.area.clear();
	
	mission.__set_forbiddenArea(obstacles);
}

#define	TM_HOME_0_X0		RESCALE_LON(10.0)
#define	TM_HOME_0_Y0		RESCALE_LAT(10.0)

void CMlpGuiDlg::PrepareTestMissionHomeLocations(Mission &mission)
{
	Position pos;
	std::vector<Position> homes;

	pos.__set_longitude(TM_HOME_0_X0);
	pos.__set_latitude(TM_HOME_0_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	homes.push_back(pos);

	mission.__set_homeLocation(homes);
}

#define	TM_VEHICLE_0_ID		1
#define	TM_VEHICLE_0_X0		RESCALE_LON(100.0)
#define	TM_VEHICLE_0_Y0		RESCALE_LAT(100.0)

void CMlpGuiDlg::PrepareTestMissionVehicles(Mission &mission)
{
	Vehicle vehicle;
	std::vector<Vehicle> vehicles;
	Position pos;
	StateVector sv;

	vehicle.__set_id(TM_VEHICLE_0_ID);
	vehicle.__set_type(VehicleType::UAV);
	vehicle.__set_maxSpeed(5);
	pos.__set_longitude(TM_VEHICLE_0_X0);
	pos.__set_latitude(TM_VEHICLE_0_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	sv.__set_position(pos);
	vehicle.__set_stateVector(sv);
	vehicles.push_back(vehicle);

	mission.__set_vehicles(vehicles);
}

#define	TM_TASK_0_X0		RESCALE_LON(200.0)
#define	TM_TASK_0_Y0		RESCALE_LAT(200.0)
#define	TM_TASK_0_X1		RESCALE_LON(390.0)
#define	TM_TASK_0_Y1		RESCALE_LAT(390.0)

#define	TM_TASK_1_X0		RESCALE_LON(390.0)
#define	TM_TASK_1_Y0		RESCALE_LAT(390.0)
#define	TM_TASK_1_X1		RESCALE_LON(200.0)
#define	TM_TASK_1_Y1		RESCALE_LAT(1000.0)

#define	TM_TASK_2_X0		RESCALE_LON(200.0)
#define	TM_TASK_2_Y0		RESCALE_LAT(1000.0)
#define	TM_TASK_2_X1		RESCALE_LON(390.0)
#define	TM_TASK_2_Y1		RESCALE_LAT(810.0)

#define	TM_TASK_3_X0		RESCALE_LON(390.0)
#define	TM_TASK_3_Y0		RESCALE_LAT(810.0)
#define	TM_TASK_3_X1		RESCALE_LON(1000.0)
#define	TM_TASK_3_Y1		RESCALE_LAT(1000.0)

#define	TM_TASK_4_X0		RESCALE_LON(1000.0)
#define	TM_TASK_4_Y0		RESCALE_LAT(1000.0)
#define	TM_TASK_4_X1		RESCALE_LON(810.0)
#define	TM_TASK_4_Y1		RESCALE_LAT(810.0)

#define	TM_TASK_5_X0		RESCALE_LON(810.0)
#define	TM_TASK_5_Y0		RESCALE_LAT(810.0)
#define	TM_TASK_5_X1		RESCALE_LON(1000.0)
#define	TM_TASK_5_Y1		RESCALE_LAT(200.0)

#define	TM_TASK_6_X0		RESCALE_LON(1000.0)
#define	TM_TASK_6_Y0		RESCALE_LAT(200.0)
#define	TM_TASK_6_X1		RESCALE_LON(810.0)
#define	TM_TASK_6_Y1		RESCALE_LAT(390.0)

#define	TM_TASK_7_X0		RESCALE_LON(810.0)
#define	TM_TASK_7_Y0		RESCALE_LAT(390.0)
#define	TM_TASK_7_X1		RESCALE_LON(1300.0)
#define	TM_TASK_7_Y1		RESCALE_LAT(500.0)

#define	TM_TASK_8_X0		RESCALE_LON(1300.0)
#define	TM_TASK_8_Y0		RESCALE_LAT(450.0)
#define	TM_TASK_8_X1		RESCALE_LON(1600.0)
#define	TM_TASK_8_Y1		RESCALE_LAT(750.0)

#define	TM_TASK_9_X0		RESCALE_LON(1600.0)
#define	TM_TASK_9_Y0		RESCALE_LAT(750.0)
#define	TM_TASK_9_X1		RESCALE_LON(10.0)
#define	TM_TASK_9_Y1		RESCALE_LAT(10.0)

#define	TM_TASK_ID_BASE		100

void CMlpGuiDlg::PrepareTestMissionTasks(Mission &mission)
{
	Task task;
	TaskTemplate tt;
	std::vector<Task> tasks;
	Region taskArea;
	Position pos;

	task.__set_id(TM_TASK_ID_BASE);
	task.__set_missionId(TM_ID);
	tt.__set_taskType(TaskType::SURVEY);
	task.__set_taskTemplate(tt);
	task.__set_startTime(0);
	task.__set_endTime(1000);
	task.__set_assignedVehicleId(TM_VEHICLE_0_ID);
	task.__set_parentTaskId(0);
	task.__set_repeatCount(0);
	pos.__set_longitude(TM_TASK_0_X0);
	pos.__set_latitude(TM_TASK_0_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	pos.__set_longitude(TM_TASK_0_X1);
	pos.__set_latitude(TM_TASK_0_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	task.__set_area(taskArea);
	task.__set_speed(1.5);
	task.__set_altitude(TM_ALTITUDE);
	tasks.push_back(task);
	taskArea.area.clear();

	task.__set_id(TM_TASK_ID_BASE + 1);
	task.__set_missionId(TM_ID);
	tt.__set_taskType(TaskType::TRANSIT);
	task.__set_taskTemplate(tt);
	task.__set_startTime(1000);
	task.__set_endTime(2000);
	task.__set_assignedVehicleId(TM_VEHICLE_0_ID);
	task.__set_parentTaskId(TM_TASK_ID_BASE);
	task.__set_repeatCount(0);
	pos.__set_longitude(TM_TASK_1_X0);
	pos.__set_latitude(TM_TASK_1_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	pos.__set_longitude(TM_TASK_1_X1);
	pos.__set_latitude(TM_TASK_1_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	task.__set_area(taskArea);
	task.__set_speed(1.5);
	task.__set_altitude(TM_ALTITUDE);
	tasks.push_back(task);
	taskArea.area.clear();

	task.__set_id(TM_TASK_ID_BASE + 2);
	task.__set_missionId(TM_ID);
	tt.__set_taskType(TaskType::SURVEY);
	task.__set_taskTemplate(tt);
	task.__set_startTime(2000);
	task.__set_endTime(3000);
	task.__set_assignedVehicleId(TM_VEHICLE_0_ID);
	task.__set_parentTaskId(TM_TASK_ID_BASE + 1);
	task.__set_repeatCount(0);
	pos.__set_longitude(TM_TASK_2_X0);
	pos.__set_latitude(TM_TASK_2_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	pos.__set_longitude(TM_TASK_2_X1);
	pos.__set_latitude(TM_TASK_2_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	task.__set_area(taskArea);
	task.__set_speed(1.5);
	task.__set_altitude(TM_ALTITUDE);
	tasks.push_back(task);
	taskArea.area.clear();

	task.__set_id(TM_TASK_ID_BASE + 3);
	task.__set_missionId(TM_ID);
	tt.__set_taskType(TaskType::TRANSIT);
	task.__set_taskTemplate(tt);
	task.__set_startTime(3000);
	task.__set_endTime(4000);
	task.__set_assignedVehicleId(TM_VEHICLE_0_ID);
	task.__set_parentTaskId(TM_TASK_ID_BASE + 2);
	task.__set_repeatCount(0);
	pos.__set_longitude(TM_TASK_3_X0);
	pos.__set_latitude(TM_TASK_3_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	pos.__set_longitude(TM_TASK_3_X1);
	pos.__set_latitude(TM_TASK_3_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	task.__set_area(taskArea);
	task.__set_speed(1.5);
	task.__set_altitude(TM_ALTITUDE);
	tasks.push_back(task);
	taskArea.area.clear();

	task.__set_id(TM_TASK_ID_BASE + 4);
	task.__set_missionId(TM_ID);
	tt.__set_taskType(TaskType::SURVEY);
	task.__set_taskTemplate(tt);
	task.__set_startTime(4000);
	task.__set_endTime(5000);
	task.__set_assignedVehicleId(TM_VEHICLE_0_ID);
	task.__set_parentTaskId(TM_TASK_ID_BASE + 3);
	task.__set_repeatCount(0);
	pos.__set_longitude(TM_TASK_4_X0);
	pos.__set_latitude(TM_TASK_4_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	pos.__set_longitude(TM_TASK_4_X1);
	pos.__set_latitude(TM_TASK_4_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	task.__set_area(taskArea);
	task.__set_speed(1.5);
	task.__set_altitude(TM_ALTITUDE);
	tasks.push_back(task);
	taskArea.area.clear();

	task.__set_id(TM_TASK_ID_BASE + 5);
	task.__set_missionId(TM_ID);
	tt.__set_taskType(TaskType::TRANSIT);
	task.__set_taskTemplate(tt);
	task.__set_startTime(5000);
	task.__set_endTime(6000);
	task.__set_assignedVehicleId(TM_VEHICLE_0_ID);
	task.__set_parentTaskId(TM_TASK_ID_BASE + 4);
	task.__set_repeatCount(0);
	pos.__set_longitude(TM_TASK_5_X0);
	pos.__set_latitude(TM_TASK_5_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	pos.__set_longitude(TM_TASK_5_X1);
	pos.__set_latitude(TM_TASK_5_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	task.__set_area(taskArea);
	task.__set_speed(1.5);
	task.__set_altitude(TM_ALTITUDE);
	tasks.push_back(task);
	taskArea.area.clear();

	task.__set_id(TM_TASK_ID_BASE + 6);
	task.__set_missionId(TM_ID);
	tt.__set_taskType(TaskType::SURVEY);
	task.__set_taskTemplate(tt);
	task.__set_startTime(6000);
	task.__set_endTime(7000);
	task.__set_assignedVehicleId(TM_VEHICLE_0_ID);
	task.__set_parentTaskId(TM_TASK_ID_BASE + 5);
	task.__set_repeatCount(0);
	pos.__set_longitude(TM_TASK_6_X0);
	pos.__set_latitude(TM_TASK_6_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	pos.__set_longitude(TM_TASK_6_X1);
	pos.__set_latitude(TM_TASK_6_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	task.__set_area(taskArea);
	task.__set_speed(1.5);
	task.__set_altitude(TM_ALTITUDE);
	tasks.push_back(task);
	taskArea.area.clear();

	task.__set_id(TM_TASK_ID_BASE + 7);
	task.__set_missionId(TM_ID);
	tt.__set_taskType(TaskType::TRANSIT);
	task.__set_taskTemplate(tt);
	task.__set_startTime(7000);
	task.__set_endTime(8000);
	task.__set_assignedVehicleId(TM_VEHICLE_0_ID);
	task.__set_parentTaskId(TM_TASK_ID_BASE + 6);
	task.__set_repeatCount(0);
	pos.__set_longitude(TM_TASK_7_X0);
	pos.__set_latitude(TM_TASK_7_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	pos.__set_longitude(TM_TASK_7_X1);
	pos.__set_latitude(TM_TASK_7_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	task.__set_area(taskArea);
	task.__set_speed(1.5);
	task.__set_altitude(TM_ALTITUDE);
	tasks.push_back(task);
	taskArea.area.clear();

	task.__set_id(TM_TASK_ID_BASE + 8);
	task.__set_missionId(TM_ID);
	tt.__set_taskType(TaskType::SURVEY);
	task.__set_taskTemplate(tt);
	task.__set_startTime(8000);
	task.__set_endTime(9000);
	task.__set_assignedVehicleId(TM_VEHICLE_0_ID);
	task.__set_parentTaskId(TM_TASK_ID_BASE + 7);
	task.__set_repeatCount(0);
	pos.__set_longitude(TM_TASK_8_X0);
	pos.__set_latitude(TM_TASK_8_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	pos.__set_longitude(TM_TASK_8_X1);
	pos.__set_latitude(TM_TASK_8_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	task.__set_area(taskArea);
	task.__set_speed(1.5);
	task.__set_altitude(TM_ALTITUDE);
	tasks.push_back(task);
	taskArea.area.clear();

	task.__set_id(TM_TASK_ID_BASE + 9);
	task.__set_missionId(TM_ID);
	tt.__set_taskType(TaskType::TRANSIT);
	task.__set_taskTemplate(tt);
	task.__set_startTime(9000);
	task.__set_endTime(10000);
	task.__set_assignedVehicleId(TM_VEHICLE_0_ID);
	task.__set_parentTaskId(TM_TASK_ID_BASE + 8);
	task.__set_repeatCount(0);
	pos.__set_longitude(TM_TASK_9_X0);
	pos.__set_latitude(TM_TASK_9_Y0);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	pos.__set_longitude(TM_TASK_9_X1);
	pos.__set_latitude(TM_TASK_9_Y1);
	pos.__set_altitude(TM_ALTITUDE);
	taskArea.area.push_back(pos);
	task.__set_area(taskArea);
	task.__set_speed(1.5);
	task.__set_altitude(TM_ALTITUDE);
	tasks.push_back(task);
	taskArea.area.clear();

	mission.__set_tasks(tasks);
}

extern	void LogMission(std::string prefix, int32_t requestId, const Mission& context, PlannerHelper& helper);

void CMlpGuiDlg::PrepareTestMission()
{
	Mission testMission;

	testMission.__set_missionId(TM_ID);
	testMission.__set_name("Test Mission");
	PrepareTestMissionNavArea(testMission);
	PrepareTestMissionForbiddenAreas(testMission);
	PrepareTestMissionHomeLocations(testMission);
	PrepareTestMissionVehicles(testMission);
	PrepareTestMissionTasks(testMission);

	PlannerHelper plannerHelper(this);

	::PostMessage(this->m_hWnd, WM_MLP_STATUS, WP_MLP_START, 0);
	::PostMessage(this->m_hWnd, WM_MLP_INFO, 0, (LPARAM)PlannerHelper::formatAndAllocateString(_T("Computing plan ...")));
	plannerHelper.preparePlan(testMission);

	LogMission("OUT_", 5678, testMission, plannerHelper);
}
