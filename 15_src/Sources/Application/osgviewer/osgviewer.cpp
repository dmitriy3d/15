/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2010 Robert Osfield
 *
 * This application is open source and may be redistributed and/or modified
 * freely and without restriction, both in commercial and non commercial applications,
 * as long as this copyright notice is maintained.
 *
 * This application is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>

#include <osg/Switch>
#include <osg/Types>
#include <osgText/Text>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>

#include <osgGA/Device>

#include <iostream>


#include <osgViewer/api/Win32/GraphicsWindowWin32>
#include <osg/PositionAttitudeTransform>

#include <random>

/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define BAR_SIZE 3.937

void GetStartDir(LPTSTR lpstrStartDir)
{
	int i;

	GetModuleFileName(NULL, lpstrStartDir, MAX_PATH + 1);
	i = lstrlen(lpstrStartDir) - 1;
	while (i >= 0)
	{
		if (lpstrStartDir[i] == TEXT('\\')) break;
		else i--;
	}
	i++;
	lpstrStartDir[i] = TEXT('\0');
}

class PickHandler : public osgGA::GUIEventHandler
{
public:
	PickHandler() {}
	PickHandler(osgViewer::Viewer& viewer);
	~PickHandler() {}

	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	virtual void pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);

	void AddBar(int i, osg::Node* pBar) { m_pBars[i] = pBar; }
	osg::Node* GetBar(int i) { return m_pBars[i]; }
	void SetRootNode(osg::Node* pRoot) { m_pRoot = pRoot; }
	void AddTransformation();
	void GenerateField();
	void MoveBar(int iID);
	bool IsMooving();
	int PickBar(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);
	osg::Vec3d GetMoveDirection(std::pair<int, int> pos, std::pair<int, int> newpos);
	std::pair<int, int> GetBarPos(int iID);
	std::pair<int, int> GetNewBarPos(std::pair<int, int> pos);

protected:

	std::map<int, osg::Node*> m_pBars;
	std::map<int, osg::ref_ptr<osg::PositionAttitudeTransform>> m_pPAT;
	osg::Node* m_pRoot;
	osg::ref_ptr<osg::AnimationPath>         m_AP;
	osg::ref_ptr<osg::AnimationPathCallback> m_APCB;
	int m_iField[4][4];
	int m_iBar;
	DWORD m_dwMoveTime;
};

PickHandler::PickHandler(osgViewer::Viewer& viewer)
:m_AP(nullptr)
,m_dwMoveTime(0)
,m_iBar(-1)
{
	auto root = viewer.getSceneData();

	SetRootNode(root);

	for (int i = 0; i < 15; i++) AddBar(i, root->asGroup()->getChild(i));

	GenerateField();
	AddTransformation();
}

bool PickHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (ea.getEventType() == osgGA::GUIEventAdapter::DOUBLECLICK)
	{
		osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);

		if (view) pick(view, ea);
	}

	return false;
}

bool PickHandler::IsMooving()
{
	if (m_AP.valid())
	{
		if ((GetTickCount() - m_dwMoveTime) >= 600)
		{
			if (m_iBar != -1)
			{
				m_pPAT[m_iBar]->setUpdateCallback(nullptr);
				m_AP.release();
				m_APCB.release();
				m_iBar = -1;
			}
		}
		else return true;
	}

	return false;
}

int PickHandler::PickBar(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
	osgUtil::LineSegmentIntersector::Intersections intersections;
	int  iID = -1;
	char sz1[256];
	char sz2[256];

	if (view->computeIntersections(ea, intersections))
	{
		auto intersection = intersections.begin();
		std::string sName = intersection->nodePath.back()->getName();

		if (!intersection->nodePath.empty() && !(sName.empty()))
		{
			for (int i = 1; i <= 15; i++)
			{
				wsprintf(sz1, "ChamferBox%03i", i);
				wsprintf(sz2, "Text%03i", i);

				if ((sName.compare(0, lstrlen(sz1), sz1) == 0) || (sName.compare(0, lstrlen(sz2), sz2) == 0))
				{
					iID = i - 1;
					break;
				}
			}
		}
	}

	return iID;
}

void PickHandler::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
	if (!IsMooving()) MoveBar(PickBar(view, ea));
}

void PickHandler::GenerateField()
{
	std::random_device         rd;
	std::default_random_engine generator(rd());
	std::vector<int>           iPos = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 };

	int iN;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			std::uniform_int_distribution<int> distribution(0, 15 - (i*4+j));

			iN = distribution(generator);

			m_iField[i][j] = iPos[iN];

			iPos.erase(std::find(iPos.begin(), iPos.end(), iPos[iN]));
		}
	}
}

void PickHandler::AddTransformation()
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (m_iField[i][j] != 15)
			{
				m_pPAT[m_iField[i][j]] = new osg::PositionAttitudeTransform();

				m_pRoot->asGroup()->addChild(m_pPAT[m_iField[i][j]]);
				m_pPAT[m_iField[i][j]]->addChild(GetBar(m_iField[i][j]));
				m_pRoot->asGroup()->removeChild(GetBar(m_iField[i][j]));

				m_pPAT[m_iField[i][j]]->setPosition(osg::Vec3d((double)i * BAR_SIZE, -(double)j * BAR_SIZE, 0.0));
			}
		}
	}
}

void PickHandler::MoveBar(int iID)
{
	if (iID == -1) return;

	std::pair<int, int> pos    = GetBarPos(iID);
	std::pair<int, int> newpos = GetNewBarPos(pos);

	osg::Vec3d v = GetMoveDirection(pos,newpos);

	if ((v._v[0] != 0.0) || (v._v[1] != 0.0))
	{
		m_iBar = iID;

		m_AP   = new osg::AnimationPath();
		m_APCB = new osg::AnimationPathCallback();

		m_AP->setLoopMode(osg::AnimationPath::NO_LOOPING);

		m_AP->insert(0.0, osg::AnimationPath::ControlPoint(osg::Vec3d((double)pos.first * BAR_SIZE, - (double)pos.second * BAR_SIZE, 0.0)));
		m_AP->insert(0.5, osg::AnimationPath::ControlPoint(osg::Vec3d((double)pos.first * BAR_SIZE, - (double)pos.second * BAR_SIZE, 0.0) + v));

		m_APCB->setAnimationPath(m_AP.get());

		m_pPAT[iID]->setUpdateCallback(m_APCB.get());

		std::swap(m_iField[pos.first][pos.second],m_iField[newpos.first][newpos.second]);

		m_dwMoveTime = GetTickCount();
	}
}

std::pair<int, int> PickHandler::GetBarPos(int iID)
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (m_iField[i][j] == iID) return std::make_pair(i, j);
		}
	}
			
	return std::make_pair(-1,-1);
}

std::pair<int, int> PickHandler::GetNewBarPos(std::pair<int, int> pos)
{
	if ((pos.first - 1) >= 0)
	{
		if (m_iField[pos.first - 1][pos.second] == 15) return std::make_pair(pos.first-1,pos.second);
	}

	if ((pos.second - 1) >= 0)
	{
		if (m_iField[pos.first][pos.second - 1] == 15) return std::make_pair(pos.first, pos.second-1);
	}

	if ((pos.first + 1) < 4)
	{
		if (m_iField[pos.first + 1][pos.second] == 15) return std::make_pair(pos.first+1, pos.second);
	}

	if ((pos.second + 1) < 4)
	{
		if (m_iField[pos.first][pos.second + 1] == 15) return std::make_pair(pos.first, pos.second+1);
	}

	return std::make_pair(-1, -1);
}

osg::Vec3d PickHandler::GetMoveDirection(std::pair<int, int> pos, std::pair<int, int> newpos)
{
	if((newpos.first == -1) || (newpos.second == -1)) return osg::Vec3d(0.0, 0.0, 0.0);

	std::pair<int, int> dir = std::make_pair(newpos.first - pos.first, newpos.second - pos.second);

	return osg::Vec3d((double)dir.first * BAR_SIZE, -(double)dir.second * BAR_SIZE, 0.0);
}
//////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
	char szStartDir[MAX_PATH + 1];
	char szFileName[MAX_PATH + 1];

	GetStartDir(szStartDir);

	wsprintf(szFileName, "%sscene\\15.osg", szStartDir);
	LPSTR args[6] = { "15",szFileName,"--samples","4","--screen","0" };
	int argn = 6;


//	osg::ArgumentParser arguments(&argc, argv);
	osg::ArgumentParser arguments(&argn, args);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is the standard OpenSceneGraph example which loads and visualises 3d models.");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");
    arguments.getApplicationUsage()->addCommandLineOption("--image <filename>","Load an image and render it on a quad");
    arguments.getApplicationUsage()->addCommandLineOption("--dem <filename>","Load an image/DEM and render it on a HeightField");
    arguments.getApplicationUsage()->addCommandLineOption("--login <url> <username> <password>","Provide authentication information for http file access.");
    arguments.getApplicationUsage()->addCommandLineOption("-p <filename>","Play specified camera path animation file, previously saved with 'z' key.");
    arguments.getApplicationUsage()->addCommandLineOption("--speed <factor>","Speed factor for animation playing (1 == normal speed).");
    arguments.getApplicationUsage()->addCommandLineOption("--device <device-name>","add named device to the viewer");

    osgViewer::Viewer viewer(arguments);

    unsigned int helpType = 0;
    if ((helpType = arguments.readHelpType()))
    {
        arguments.getApplicationUsage()->write(std::cout, helpType);
        return 1;
    }

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }

    if (arguments.argc()<=1)
    {
        arguments.getApplicationUsage()->write(std::cout,osg::ApplicationUsage::COMMAND_LINE_OPTION);
        return 1;
    }

    std::string url, username, password;
    while(arguments.read("--login",url, username, password))
    {
        if (!osgDB::Registry::instance()->getAuthenticationMap())
        {
            osgDB::Registry::instance()->setAuthenticationMap(new osgDB::AuthenticationMap);
            osgDB::Registry::instance()->getAuthenticationMap()->addAuthenticationDetails(
                url,
                new osgDB::AuthenticationDetails(username, password)
            );
        }
    }

    std::string device;
    while(arguments.read("--device", device))
    {
        osg::ref_ptr<osgGA::Device> dev = osgDB::readRefFile<osgGA::Device>(device);
        if (dev.valid())
        {
            viewer.addDevice(dev);
        }
    }

    // set up the camera manipulators.
    {
        /*osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

        keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
        keyswitchManipulator->addMatrixManipulator( '2', "Flight", new osgGA::FlightManipulator() );
        keyswitchManipulator->addMatrixManipulator( '3', "Drive", new osgGA::DriveManipulator() );
        keyswitchManipulator->addMatrixManipulator( '4', "Terrain", new osgGA::TerrainManipulator() );
        keyswitchManipulator->addMatrixManipulator( '5', "Orbit", new osgGA::OrbitManipulator() );
        keyswitchManipulator->addMatrixManipulator( '6', "FirstPerson", new osgGA::FirstPersonManipulator() );
        keyswitchManipulator->addMatrixManipulator( '7', "Spherical", new osgGA::SphericalManipulator() );

        std::string pathfile;
        double animationSpeed = 1.0;
        while(arguments.read("--speed",animationSpeed) ) {}
        char keyForAnimationPath = '8';
        while (arguments.read("-p",pathfile))
        {
            osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
            if (apm || !apm->valid())
            {
                apm->setTimeScale(animationSpeed);

                unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
                keyswitchManipulator->addMatrixManipulator( keyForAnimationPath, "Path", apm );
                keyswitchManipulator->selectMatrixManipulator(num);
                ++keyForAnimationPath;
            }
        }*/

//		viewer.setCameraManipulator(keyswitchManipulator.get());
//		viewer.setCameraManipulator(new osgGA::SphericalManipulator());
		viewer.setCameraManipulator(new osgGA::OrbitManipulator());
	}

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    // add the thread model handler
    viewer.addEventHandler(new osgViewer::ThreadingHandler);

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    // add the help handler
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    // add the record camera path handler
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

    // add the LOD Scale handler
    viewer.addEventHandler(new osgViewer::LODScaleHandler);

    // add the screen capture handler
    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);

    // load the data
    osg::ref_ptr<osg::Node> loadedModel = osgDB::readRefNodeFiles(arguments);
    if (!loadedModel)
    {
        std::cout << arguments.getApplicationName() <<": No data loaded" << std::endl;
        return 1;
    }

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }


    // optimize the scene graph, remove redundant nodes and state etc.
    osgUtil::Optimizer optimizer;
    optimizer.optimize(loadedModel);

    viewer.setSceneData(loadedModel);

    viewer.realize();

	
/////////////////////////////////////////////////////////////////////////////////

	osgGA::OrbitManipulator* cm = (osgGA::OrbitManipulator*)viewer.getCameraManipulator();
	cm->setTransformation(osg::Vec3d(5.9, -5.9, 50.0), osg::Vec3d(5.9, -5.9, 0.0), osg::Vec3d(0.0, 1.0, 0.0));

	osg::ref_ptr<PickHandler> ph = new PickHandler(viewer);

	viewer.addEventHandler(ph.get());

    return viewer.run();
}