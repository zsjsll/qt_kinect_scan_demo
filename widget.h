#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "Kinect.h"
#include <NuiKinectFusionApi.h>
#include "d2d1.h"

#pragma comment ( lib, "d2d1.lib" )
#pragma comment ( lib, "windowscodecs.lib" )
#pragma comment ( lib, "dwmapi.lib" )
#pragma comment ( lib, "Msdmo.lib" )
#pragma comment ( lib, "dmoguids.lib" )
#pragma comment ( lib, "amstrmid.lib" )
#pragma comment ( lib, "avrt.lib" )
#pragma comment ( lib, "dwrite.lib" )
#pragma comment ( lib, "kinect20.lib" )
#pragma comment ( lib, "Kinect20.Fusion.lib" )

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();
private slots:
    void updateKinectData();
	void debugButton();
	void colorcam();
	
	
private:
    void init();

	void check_depth_frame();
	Ui::Widget *ui;
	ICoordinateMapper* m_pMapper = NULL;
	NUI_FUSION_RECONSTRUCTION_PARAMETERS m_reconstructionParams;
	NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE m_processorType;
	INT m_deviceIndex;
	Matrix4 m_worldToCameraTransform;
	INuiFusionReconstruction* m_pReconstruction;
	Matrix4 m_defaultWorldToVolumeTransform;

	UINT m_cDepthWidth;
	UINT m_cDepthHeight;
	NUI_FUSION_IMAGE_FRAME* m_pDepthFloatImage;
	NUI_FUSION_IMAGE_FRAME* m_pSmoothDepthFloatImage;
	NUI_FUSION_IMAGE_FRAME* m_pPointCloud;
	NUI_FUSION_IMAGE_FRAME* m_pSurfaceImageFrame;
	NUI_FUSION_IMAGE_FRAME* m_pNormalImageFrame;


	IColorFrameSource* pColorFrameSource = NULL;
	IDepthFrameSource* pDepthFrameSource = NULL;
	IDepthFrameReader* m_pDepthFrameReader = NULL;
	float m_fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;
	// 深度最远阈值
	float m_fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;

	USHORT m_cMaxIntegrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;


	// 缓存位图帧
	ID2D1Bitmap*                        m_pDrawBitmap = nullptr;
	// 表面位图帧
	ID2D1Bitmap*                        m_pSurfaceBitmap = nullptr;
	// 法线位图帧
	ID2D1Bitmap*                        m_pNormalBitmap = nullptr;

};

#endif // WIDGET_H
