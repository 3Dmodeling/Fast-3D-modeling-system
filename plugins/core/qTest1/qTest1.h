//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qBlur                       #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef Q_BLUR_PLUGIN_HEADER
#define Q_BLUR_PLUGIN_HEADER

#include "../ccStdPluginInterface.h"
//qCC
#include <ccCommon.h> 

//qCC_db
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccGenericMesh.h>
#include <ccPlane.h>
#include <ccSphere.h>
#include <ccCylinder.h>
#include <ccCone.h>
#include <ccTorus.h>
#include <ccScalarField.h>

//CCLib
#include <ScalarField.h>
#include <CCPlatform.h>

//Qt
#include <QtGui> 
#include <QtConcurrentRun> 
#include <QtWidgets>

class qTest1;
class Test1Dialog : public QDialog
{
	Q_OBJECT
public:
	Test1Dialog(QWidget *pMainWindow, qTest1 *pApp);
signals: 
private:
	qTest1 *m_pPlugin = nullptr;
};

class tScalarField : public ccScalarField
{
public:
	int m_iBinCount = 360;
	void computeNewHistorgram();
	Histogram m_newHistogram;
	std::vector<unsigned int> m_aMapIdx2Bin;	
};
//! test plugin
class qTest1 : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qTest1")

public:

	//inherited from ccPluginInterface
	virtual QString getName() const override { return "Test1 plugin"; }
	virtual QString getDescription() const override { return "ground blueprint"; }
	virtual QIcon getIcon() const override;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities);
	virtual void getActions(QActionGroup& group);
	 
	public slots :
	void onGroundProjection(); 
	void onFindDominantNormals();
	void onFilterRANSACPlanes();
	void onFindGround();
	void onFindRoof();
	void onFindWalls();
	//! Slot called when associated ation is triggered
	void doAction();

protected:

	//! Associated action
	QAction* m_action = nullptr;
	Test1Dialog *m_pTestDialog = nullptr;
	std::vector<CCVector3> m_aNormal;
};

#endif
