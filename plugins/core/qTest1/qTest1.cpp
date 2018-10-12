


#include "qTest1.h"

const QString g_sSFNameDipDirDeg360 = "DipDir Degree 360";
const QString g_sSFNameDipDeg90 = "Dip Degree 90";

Test1Dialog::Test1Dialog(QWidget *pMainWindow, qTest1 *pPlugin)
	: QDialog(pMainWindow), m_pPlugin(pPlugin) {

	QVBoxLayout *mainLayout = new QVBoxLayout;
	setLayout(mainLayout);
	setWindowTitle(u8"Test Dialog");

	QPushButton *pPB1 = new QPushButton(u8"地面投影");
	mainLayout->addWidget(pPB1);
	connect(pPB1, SIGNAL(clicked()), m_pPlugin, SLOT(onGroundProjection()));
	 
	pPB1 = new QPushButton(u8"找主法向");
	mainLayout->addWidget(pPB1);
	connect(pPB1, SIGNAL(clicked()), m_pPlugin, SLOT(onFindDominantNormals()));

	mainLayout->addWidget(new QLabel(u8"用RANSAC求候选平面"));

	pPB1 = new QPushButton(u8"过滤候选平面");
	mainLayout->addWidget(pPB1);
	connect(pPB1, SIGNAL(clicked()), m_pPlugin, SLOT(onFilterRANSACPlanes()));

	pPB1 = new QPushButton(u8"找地面");
	mainLayout->addWidget(pPB1);
	connect(pPB1, SIGNAL(clicked()), m_pPlugin, SLOT(onFindGround()));

	pPB1 = new QPushButton(u8"找屋顶");
	mainLayout->addWidget(pPB1);
	connect(pPB1, SIGNAL(clicked()), m_pPlugin, SLOT(onFindRoof()));

	pPB1 = new QPushButton(u8"找墙面");
	mainLayout->addWidget(pPB1);
	connect(pPB1, SIGNAL(clicked()), m_pPlugin, SLOT(onFindWalls()));



}

//! Default number of classes for associated histogram
void tScalarField::computeNewHistorgram()
{
	ScalarField::computeMinAndMax();
	m_displayRange.setBounds(m_minVal, m_maxVal);
	//update histogram
	{
		if (m_displayRange.maxRange() == 0 || currentSize() == 0)
		{
			//can't build histogram of a flat field
			m_newHistogram.clear();
		}
		else
		{
			unsigned count = currentSize();
			unsigned numberOfClasses = static_cast<unsigned>(ceil(sqrt(static_cast<double>(count))));
			numberOfClasses = std::max<unsigned>(std::min<unsigned>(numberOfClasses, m_iBinCount), 4);

			m_newHistogram.maxValue = 0;

			//reserve memory
			try
			{
				m_newHistogram.resize(numberOfClasses);
			}
			catch (const std::bad_alloc&)
			{
				ccLog::Warning("[ccScalarField::computeMinAndMax] Failed to update associated histogram!");
				m_newHistogram.clear();
			}

			if (!m_newHistogram.empty())
			{
				std::fill(m_newHistogram.begin(), m_newHistogram.end(), 0);
				
				m_aMapIdx2Bin.resize(count); // renc

				//compute histogram
				{
					ScalarType step = static_cast<ScalarType>(numberOfClasses) / m_displayRange.maxRange();
					for (unsigned i = 0; i < count; ++i)
					{
						const ScalarType& val = getValue(i);

						unsigned bin = static_cast<unsigned>(floor((val - m_displayRange.min())*step));
						bin = std::min(bin, numberOfClasses - 1);
						++m_newHistogram[bin]; // renc: 统计个数

						m_aMapIdx2Bin[i] = bin;
					}
				}

				//update 'maxValue'
				m_newHistogram.maxValue = *std::max_element(m_newHistogram.begin(), m_newHistogram.end());
			}
		}
	}

	m_modified = true; 
}

//------------------------------------------------------
QIcon qTest1::getIcon() const
{
    return QIcon();
}

void qTest1::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
		m_action->setEnabled(selectedEntities.size() == 1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
}

void qTest1::getActions(QActionGroup& group)
{
	//default action
	if (!m_action)
	{
		m_action = new QAction(getName(), this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		//connect signal
		connect(m_action, SIGNAL(triggered()), this, SLOT(doAction()));
	}

	group.addAction(m_action);
}

void qTest1::doAction()
{
	assert(m_app);
	if (!m_app)
		return;
	
	if (m_pTestDialog) {
		m_pTestDialog->show();
	}
	else {
		m_pTestDialog = new Test1Dialog(m_app->getMainWindow(), this);
		m_pTestDialog->show();
	}
}


void qTest1::onGroundProjection()
{

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
	if (selNum != 1)
	{
		m_app->dispToConsole("Select only one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccHObject* ent = selectedEntities[0];
	assert(ent);
	if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select a real point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* pc = static_cast<ccPointCloud*>(ent);

	ccPointCloud *pcCopy = new ccPointCloud("copied");
	if (!pcCopy->reserve((unsigned)pc->size()))
	{
		m_app->dispToConsole("Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		delete pcCopy;
		return;
	}

	for (int i = 0; i < pc->size(); ++i) {
		CCVector3 pos = *pc->getPoint(i);
		pos[2] = 0;
		pcCopy->addPoint(pos);
	}


	//random color
	ccColor::Rgb col = ccColor::Generator::Random();
	pcCopy->setRGBColor(col);
	pcCopy->showColors(true);
	pcCopy->setVisible(true);

	ccHObject *group = new ccHObject(QString("Copy (%1)").arg(ent->getName()));
	group->addChild(pcCopy);
	if (group)
	{
		assert(group->getChildrenNumber() != 0);

		//we add new group to DB/display
		group->setVisible(true);
		group->setDisplay_recursive(pc->getDisplay());
		m_app->addToDB(group);

		m_app->refreshAll();
	}
}
void findMaxElement(const std::vector< std::vector<unsigned int > > &histogram, 
	int &iCount, int &x, int &y) {

	int iCount1 = 0, x1 = 0, y1 = 0;
	for (int i = 0; i < histogram.size(); ++i) {
		for (int j = 0; j < histogram[i].size(); ++j)
		{
			unsigned int iVal = histogram[i][j];
			if (iVal > iCount1) {
				iCount1 = iVal; x1 = i; y1 = j;
			}
		}
	}
	iCount = iCount1; x = x1; y = y1;
}
void flatElement(std::vector<std::vector<unsigned int> > &histogram, int x, int y, int range=5)
{ 
	int xStart = x - range; //if (xStart < 0) xStart = 0;
	int xEnd = x + range; //if (xEnd > histogram.size()) xEnd = histogram.size();
	int yStart = y - range; if (yStart < 0) yStart = 0;
	int yEnd = y + range; if (yEnd > histogram[0].size()) yEnd = histogram[0].size();
	for (int i = xStart; i < xEnd; ++i) {
		int ii = i;
		if (i < 0) ii = i + 360;
		else if (i >= 360) ii = i - 360;
		for (int j = yStart; j < yEnd; ++j) {
			histogram[ii][j] = 0;
		}
	}
}
void histogramDebugOutput(const std::vector<std::vector<unsigned int> > &histogram, 
	int iMaxCount, QString sFilePath)
{
	QImage imgHistogramOri(360, 90, QImage::Format_RGBA8888);
	for (int i = 0; i < histogram.size(); ++i) {
		for (int j = 0; j < histogram[i].size(); ++j)
		{
			unsigned int iVal = histogram[i][j];
			unsigned int r = iVal*1.0 / iMaxCount * 255;
			imgHistogramOri.setPixelColor(i, j, QColor(r, 0, 0));
		}
	}
	imgHistogramOri.save(sFilePath);
	QString sCsvFilePath = sFilePath+".csv";
	QFile fileOri(sCsvFilePath);
	if (!fileOri.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		qInfo() << "Fail to open file " << sCsvFilePath;
	}
	QTextStream out(&fileOri);
	for (int i = 0; i < histogram.size(); ++i) {
		for (int j = 0; j < histogram[i].size(); ++j)
		{
			unsigned int iVal = histogram[i][j];
			out << iVal << ", ";
		}
		out << "\n";
	}
	fileOri.close();
}
double angleDegree(CCVector3 u, CCVector3 v) {
	return std::acos(u.dot(v)) * 180.0 / std::_Pi;
}
void invertNormal(CCVector3 &d) {
	double x = fabs(d[0]), y = fabs(d[1]), z = fabs(d[2]);
	// 绝对值最大的元素 < 0
	if ((x > y && x > z && d[0] < 0) || 
		(y > x && y > z && d[1] < 0) || 
		(z > x && z > y && d[2] < 0)) 
	{
		d *= -1;
	}
}
void qTest1::onFindDominantNormals()
{
	m_aNormal.clear();

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
	if (selNum != 1)
	{
		m_app->dispToConsole("Select only one cloud! End.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccHObject* ent = selectedEntities[0];
	assert(ent);
	if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select a real point cloud! End.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* pc = static_cast<ccPointCloud*>(ent);

	if (pc->hasNormals()) {
		for (int i = 0; i < pc->size(); ++i) {
			CCVector3 pos = *pc->getPoint(i);
			CCVector3 nor = pc->getPointNormal(i);
		}
	}
	else {
		m_app->dispToConsole("Without normals in this point cloud! End.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//get/create 'dip direction' scalar field
	int dipDirSFIndex = pc->getScalarFieldIndexByName(CC_DEFAULT_DIP_DIR_SF_NAME);
	if (dipDirSFIndex < 0)
		dipDirSFIndex = pc->addScalarField(CC_DEFAULT_DIP_DIR_SF_NAME);
	if (dipDirSFIndex < 0)
	{
		m_app->dispToConsole("Without DIP_DIR_SF in this point cloud! End.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	ccScalarField* dipDirSF = (ccScalarField*)(pc->getScalarField(dipDirSFIndex));
	int dipSFIndex = pc->getScalarFieldIndexByName(CC_DEFAULT_DIP_SF_NAME);
	if (dipSFIndex < 0)
		dipSFIndex = pc->addScalarField(CC_DEFAULT_DIP_SF_NAME);
	if (dipSFIndex < 0)
	{
		ccLog::Warning("[ccEntityAction::convertNormalsTo] Not enough memory!");
		return;
	}
	ccScalarField* dipSF = static_cast<ccScalarField*>(pc->getScalarField(dipSFIndex));

	if (dipDirSF == nullptr || dipSF == nullptr) {
		m_app->dispToConsole("Without DIP_DIR_SF in this point cloud! End.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	else {
		std::vector< std::vector<unsigned int > > histogram;
		histogram.resize(360);
		for (int i = 0; i < histogram.size(); ++i) {
			histogram[i].resize(90); 
			for (int j = 0; j < histogram[i].size(); ++j)
				histogram[i][j] = 0;
		}
		std::vector<std::pair<unsigned int, unsigned int> > mapIdx2Bin(dipDirSF->currentSize());
		for (unsigned int i = 0; i < dipDirSF->currentSize(); ++i) {
			unsigned int u = dipDirSF->getValue(i), v = dipSF->getValue(i);
			++histogram[(u)%360][(v)%90];
			mapIdx2Bin[i] = std::pair<unsigned int, unsigned int>(u%360, v%90);
		}
		std::vector< std::vector<unsigned int > > histogramOld;
		histogramOld = histogram;

		int iCount1 = 0, x1 = 0, y1 = 0;
		findMaxElement(histogram, iCount1, x1, y1);
		histogramDebugOutput(histogram, iCount1, "D:\\histogramOri.png");

		flatElement(histogram, x1, y1, 5);
		histogramDebugOutput(histogram, iCount1, "d:\\histogramRemovePeak1.png");
		 
		int iCount2 = 0, x2 = 0, y2 = 0;
		findMaxElement(histogram, iCount2, x2, y2);
		flatElement(histogram, x2, y2);

		int iCount3 = 0, x3 = 0, y3 = 0;
		findMaxElement(histogram, iCount3, x3, y3);
		flatElement(histogram, x3, y3);
		m_app->dispToConsole(QString("Main Direction dip deg1: %1, %2").arg(x1).arg(y1));
		m_app->dispToConsole(QString("Main Direction dip deg2: %1, %2").arg(x2).arg(y2));
		m_app->dispToConsole(QString("Main Direction dip deg3: %1, %2").arg(x3).arg(y3));


		CCVector3 nor1, nor2, nor3; unsigned int iNor1 = 0, iNor2 = 0, iNor3 = 0;
		if (pc->hasNormals()) {
			for (int i = 0; i < pc->size(); ++i) { 
				unsigned int xBin = mapIdx2Bin[i].first, yBin = mapIdx2Bin[i].second;
				if (xBin == x1 && yBin == y1) {
					nor1 += pc->getPointNormal(i); ++iNor1;
				}
				else if (xBin == x2 && yBin == y2) {
					nor2 += pc->getPointNormal(i); ++iNor2;
				}
				else if (xBin == x3 && yBin == y3) {
					nor3 += pc->getPointNormal(i); ++iNor3;
				}
			}
			nor1 /= iNor1; nor2 /= iNor2; nor3 /= iNor3;
			nor1.normalize(); nor2.normalize(); nor3.normalize(); 
			m_app->dispToConsole(QString("Main Normal1: %1, %2, %3").arg(nor1[0]).arg(nor1[1]).arg(nor1[2]));
			m_app->dispToConsole(QString("Main Normal2: %1, %2, %3").arg(nor2[0]).arg(nor2[1]).arg(nor2[2]));
			m_app->dispToConsole(QString("Main Normal3: %1, %2, %3").arg(nor3[0]).arg(nor3[1]).arg(nor3[2])); 

			CCVector3 dNor1, dNor2, dNor3; // dominate 
			if (nor1.norm() > 0) {
				dNor1 = nor1;
				dNor2 = nor2; 
				double fDeg12 = angleDegree(dNor1, dNor2);
				double fDeg13 = angleDegree(dNor1, dNor3);
				if (fDeg12 < 10.0 || fDeg12 > 170.0) {
					if (fDeg13 < 10.0 || fDeg13 > 170.0)
						m_app->dispToConsole("nor1 and nor2, nor1 and nor3, are almost the same direction. ");
					else
						dNor2 = nor3;
				}  
				dNor3 = dNor1.cross(dNor2); dNor3.normalize();
			}
			invertNormal(dNor1); invertNormal(dNor2); invertNormal(dNor3);
			m_app->dispToConsole(QString("Dom Normal1: %1, %2, %3").arg(dNor1[0]).arg(dNor1[1]).arg(dNor1[2]));
			m_app->dispToConsole(QString("Dom Normal2: %1, %2, %3").arg(dNor2[0]).arg(dNor2[1]).arg(dNor2[2]));
			m_app->dispToConsole(QString("Dom Normal3: %1, %2, %3").arg(dNor3[0]).arg(dNor3[1]).arg(dNor3[2]));
			
			CCVector3 xVector(1, 0, 0);
			CCVector3 norX = dNor1, norY = dNor2, norZ = dNor3; // orient to xyz
			double fDeg1x = angleDegree(dNor1, xVector), fDeg2x = angleDegree(dNor2, xVector), fDeg3x = angleDegree(dNor3, xVector);
			m_app->dispToConsole(QString("Angle with X: %1, %2, %3").arg(fDeg1x).arg(fDeg2x).arg(fDeg3x));
			// Angle with X: 86.5337, 172.779, 90.4033, 
			if (fDeg1x > 90) {
				fDeg1x = 180 - fDeg1x; 
			}
			if (fDeg2x > 90) fDeg2x = 180 - fDeg2x;
			if (fDeg3x > 90) fDeg3x = 180 - fDeg3x;
			m_app->dispToConsole(QString("Angle with X: %1, %2, %3").arg(fDeg1x).arg(fDeg2x).arg(fDeg3x));
			if (fDeg1x < fDeg2x && fDeg1x < fDeg3x) {}
			else if (fDeg2x < fDeg1x && fDeg2x < fDeg3x) { norX = dNor2; norY = dNor3; norZ = dNor1; }
			else if (fDeg3x < fDeg1x && fDeg3x < fDeg2x) { norX = dNor3; norY = dNor1; norZ = dNor2; }
			m_app->dispToConsole(QString("Dom NormalX: %1, %2, %3").arg(norX[0]).arg(norX[1]).arg(norX[2]));
			m_app->dispToConsole(QString("Dom NormalY: %1, %2, %3").arg(norY[0]).arg(norY[1]).arg(norY[2]));
			m_app->dispToConsole(QString("Dom NormalZ: %1, %2, %3").arg(norZ[0]).arg(norZ[1]).arg(norZ[2]));
			// invert X
			double fDegX = angleDegree(norX, xVector);
			if (fDegX > 90) {
				m_app->dispToConsole("Neet to invert NormalX");
				norX *= -1;
				CCVector3 temp = norY;
				norY = norZ;
				norZ = temp;
			}

			m_app->dispToConsole(QString("Dom NormalX: %1, %2, %3").arg(norX[0]).arg(norX[1]).arg(norX[2]));
			m_app->dispToConsole(QString("Dom NormalY: %1, %2, %3").arg(norY[0]).arg(norY[1]).arg(norY[2]));
			m_app->dispToConsole(QString("Dom NormalZ: %1, %2, %3").arg(norZ[0]).arg(norZ[1]).arg(norZ[2]));

			m_aNormal.push_back(norX); m_aNormal.push_back(norY); m_aNormal.push_back(norZ);
			m_aNormal.push_back(norX * -1); m_aNormal.push_back(norY * -1); m_aNormal.push_back(norZ * -1);
			
			ccGLMatrix glMat1 = ccGLMatrix::FromToRotation(CCVector3(0, 0, 1), nor1);
			ccPlane *pPlane1 = new ccPlane(1, 1, &glMat1);
			pPlane1->setColor(ccColor::red); pPlane1->showColors(true);
			pPlane1->showNormals(true);
			//
			ccGLMatrix glMat2;
			glMat2 = ccGLMatrix::FromToRotation(CCVector3(0, 0, 1), nor2);//radians 0.7854 = 45degree.
			ccPlane *pPlane2 = new ccPlane(1, 1, &glMat2);
			pPlane2->setColor(ccColor::green); pPlane2->showColors(true);
			pPlane2->showNormalVector(true);
			//
			ccGLMatrix glMat3;
			glMat3 = ccGLMatrix::FromToRotation(CCVector3(0, 0, 1), nor3);
			ccPlane *pPlane3 = new ccPlane(1, 1, &glMat3);
			pPlane3->setColor(ccColor::blue); pPlane3->showColors(true);
			pPlane3->showNormals(true); pPlane3->showNormalVector(true);

			ccHObject *group = new ccHObject(QString(u8"主平面"));
			group->addChild(pPlane1); group->addChild(pPlane2); group->addChild(pPlane3);
			m_app->addToDB(group);
		}
		else {
			m_app->dispToConsole("Without normals in this point cloud! End.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		} 

		pc->prepareDisplayForRefresh_recursive();
	}
	m_app->dispToConsole("Finish: DipDirAngle 360 to 180.");
} 

void qTest1::onFilterRANSACPlanes()
{
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
	if (selNum != 1)
	{
		m_app->dispToConsole("Select only one cloud! End.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccHObject* ent = selectedEntities[0];
	assert(ent);
	if (!ent)
	{
		m_app->dispToConsole("Donot have a selection group! End.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	unsigned int iPCCount = ent->getChildrenNumber();
	m_app->dispToConsole(QString("Initial RANSAC planes count: %1").arg(iPCCount));
	
	ccHObject *groupCopy = 0;
	for (int iPC = 0; iPC < iPCCount; ++iPC) {
		ccHObject *child = ent->getChild(iPC);

		ccPointCloud* pcShape = static_cast<ccPointCloud*>(child);
		if (pcShape == nullptr)
			continue;

		ccPointCloud *pcShapeCopy = new ccPointCloud(pcShape->getName());
		if (!pcShapeCopy->reserve((unsigned)pcShape->size()))
		{
			m_app->dispToConsole("Not enough memory!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			delete pcShapeCopy;
			continue;
		}
		for (int i = 0; i < pcShape->size(); ++i) {
			pcShapeCopy->addPoint(*(pcShape->getPoint(i))); 
		}
		if (pcShape->hasNormals()) {
			pcShapeCopy->reserveTheNormsTable();
			for (int i = 0; i < pcShape->size(); ++i) {
				pcShapeCopy->addNorm((pcShape->getPointNormal(i)));
			}
		}
		ColorsTableType *col = pcShape->rgbColors();
		pcShapeCopy->setRGBColor(*(col->getValue(0)), *(col->getValue(1)), *(col->getValue(2)));
		pcShapeCopy->showColors(pcShape->colorsShown());
		pcShapeCopy->showNormals(true);
		pcShapeCopy->setVisible(pcShape->isVisible());
		pcShapeCopy->setGlobalShift(pcShape->getGlobalShift());
		pcShapeCopy->setGlobalScale(pcShape->getGlobalScale());

		//
		bool bFiltedOut = true;
		if (pcShape->getChildrenNumber()) {
			ccPlane *prim = (ccPlane *)pcShape->getChild(0);
			ccPlane *primCopy = (ccPlane *)prim->clone();
			primCopy->showColors(true);
			primCopy->setVisible(true);
			pcShapeCopy->addChild(primCopy);


			CCVector3 planeNormal = primCopy->getNormal();
			for (int i = 0; i < m_aNormal.size(); ++i) {
				double fDeg = angleDegree(m_aNormal[i], planeNormal);
				if (20 > fDeg || fDeg > 160)
					bFiltedOut = false;
			}
		}

		//
		if (!bFiltedOut) {
			if (!groupCopy)
				groupCopy = new ccHObject("Ransac filted");
			groupCopy->addChild(pcShapeCopy);
			//we add new group to DB/display
			groupCopy->setVisible(true);
			groupCopy->setDisplay_recursive(pcShape->getDisplay());
			m_app->addToDB(groupCopy);

			m_app->refreshAll();
		}
	}
	if (groupCopy) {
		unsigned int iPCCount = groupCopy->getChildrenNumber();
		m_app->dispToConsole(QString("After fliter, RANSAC planes count: %1").arg(iPCCount));
	}
	m_app->dispToConsole("End: filting RANSAC planes");
}

void qTest1::onFindGround()
{

}

void qTest1::onFindRoof()
{

}
void qTest1::onFindWalls()
{

}
