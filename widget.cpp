#include "widget.h"
#include "ui_widget.h"

#include <QTimer>
#include <QDebug>
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}
static const int        cColorWidth  = 1920;
static const int        cColorHeight = 1080;
IKinectSensor*          m_pKinectSensor;// Current Kinect
IColorFrameReader*      m_pColorFrameReader;// Color reader
RGBQUAD*                m_pColorRGBX;


void SetIdentityMatrix(Matrix4 &mat)
{
	mat.M11 = 1.f; mat.M12 = 0.f; mat.M13 = 0.f; mat.M14 = 0.f;
	mat.M21 = 0.f; mat.M22 = 1.f; mat.M23 = 0.f; mat.M24 = 0.f;
	mat.M31 = 0.f; mat.M32 = 0.f; mat.M33 = 1.f; mat.M34 = 0.f;
	mat.M41 = 0.f; mat.M42 = 0.f; mat.M43 = 0.f; mat.M44 = 1.f;
}



Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
	
	m_processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP;
	m_deviceIndex = -1;

	SetIdentityMatrix(m_worldToCameraTransform);
	SetIdentityMatrix(m_defaultWorldToVolumeTransform);

	m_cDepthWidth = NUI_DEPTH_RAW_WIDTH;
	m_cDepthHeight = NUI_DEPTH_RAW_HEIGHT;

	m_reconstructionParams.voxelsPerMeter = 256.f;
	m_reconstructionParams.voxelCountX = 384;
	m_reconstructionParams.voxelCountY = 384;
	m_reconstructionParams.voxelCountZ = 384;


    m_pKinectSensor = NULL;
    m_pColorFrameReader = NULL;
    m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];// create heap storage for color pixel data in RGBX format
    init();
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateKinectData()));
	connect(this->ui->pushButton, SIGNAL(clicked()), this, SLOT(debugButton()));
    timer->start(33);

}

Widget::~Widget()
{
    delete ui;
    if (m_pColorRGBX)
    {
        delete [] m_pColorRGBX;
        m_pColorRGBX = NULL;
    }

    SafeRelease(m_pColorFrameReader);// done with color frame reader

    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();// close the Kinect Sensor
    }
    SafeRelease(m_pKinectSensor);
}

void Widget::updateKinectData()
{
	//this->colorcam();
	//this->depthcam();
	this->check_depth_frame();
}

void Widget::debugButton()
{
	this->check_depth_frame();
	//this->depthcam();
}

void Widget::colorcam()
{
	if (!m_pColorFrameReader)
	{
		return;
	}

	IColorFrame* pColorFrame = NULL;

	HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
	

	if (SUCCEEDED(hr))
	{
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		RGBQUAD *pBuffer = NULL;

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
			//qDebug() << "imageformat is" << imageFormat;
		}

		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
				qDebug() << "buffer size is:" << nBufferSize;
			}
			else if (m_pColorRGBX)
			{
				pBuffer = m_pColorRGBX;
				nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
		}

		if (SUCCEEDED(hr))
		{
			//ui->label->setPixmap(QPixmap::fromImage(QImage((uchar*)pBuffer, nWidth , nHeight, QImage::Format_RGB32)).scaledToWidth(800));
			ui->label->setPixmap(QPixmap::fromImage(QImage(reinterpret_cast<uchar*>(pBuffer), nWidth, nHeight, QImage::Format_RGB32)).scaledToWidth(800));
			//ProcessColor(pBuffer, nWidth, nHeight);
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pColorFrame);
}



void Widget::init()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
	
    if (FAILED(hr))
    {
        return ;
    }
    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get the color reader

        hr = m_pKinectSensor->Open();
        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
	        
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }
        if (SUCCEEDED(hr))
        {
            hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
        }
		if (SUCCEEDED(hr)) {
			
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}
		if (SUCCEEDED(hr)) {
			
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pMapper);
		}
		if (SUCCEEDED(hr)) {
			hr = NuiFusionCreateReconstruction(
				//			重建参数
				&m_reconstructionParams,
				//cpu模式还是gpu模式
				m_processorType,
				//用了哪一个设备 默认-1
				m_deviceIndex,
				//				世界到相机的转换
				&m_worldToCameraTransform,
				//				容器输出
				&m_pReconstruction
			);
		}
		if (FAILED(hr)) { qDebug() << "error!"; }
		if (SUCCEEDED(hr)) {
			hr = m_pReconstruction->GetCurrentWorldToVolumeTransform(&m_defaultWorldToVolumeTransform);
		}
		// 创建浮点深度帧
		if (SUCCEEDED(hr)) {
			hr = NuiFusionCreateImageFrame(
				NUI_FUSION_IMAGE_TYPE_FLOAT,
				m_cDepthWidth,
				m_cDepthHeight,
				nullptr,
				&m_pDepthFloatImage
			);
		}
		// 创建平滑浮点深度帧
		if (SUCCEEDED(hr)) {
			hr = NuiFusionCreateImageFrame(
				NUI_FUSION_IMAGE_TYPE_FLOAT,
				m_cDepthWidth,
				m_cDepthHeight,
				nullptr,
				&m_pSmoothDepthFloatImage
			);
		}
		// 创建点云帧
		if (SUCCEEDED(hr)) {
			hr = NuiFusionCreateImageFrame(
				NUI_FUSION_IMAGE_TYPE_POINT_CLOUD,
				m_cDepthWidth,
				m_cDepthHeight,
				nullptr,
				&m_pPointCloud
			);
		}
		// 创建Fusion图像帧
		if (SUCCEEDED(hr)) {
			hr = NuiFusionCreateImageFrame(
				NUI_FUSION_IMAGE_TYPE_COLOR,
				m_cDepthWidth,
				m_cDepthHeight,
				nullptr,
				&m_pSurfaceImageFrame
			);
		}
		// 创建Fusion法线帧
		if (SUCCEEDED(hr)) {
			hr = NuiFusionCreateImageFrame(
				NUI_FUSION_IMAGE_TYPE_COLOR,
				m_cDepthWidth,
				m_cDepthHeight,
				nullptr,
				&m_pNormalImageFrame
			);
		}
		SafeRelease(pDepthFrameSource);
        SafeRelease(pColorFrameSource);

    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        printf("No ready Kinect found! \n");
        return;
    }
}

void Widget::check_depth_frame() {
	if (!m_pDepthFrameReader)
	{
		qDebug() << "dont have pepth frame reader!";
		return;
	}


	// 深度帧
	IDepthFrame* pDepthFrame = nullptr;
	// 帧描述
	IFrameDescription* pFrameDescription = nullptr;
	// 深度帧宽度数据
	int width = 0;
	// 深度帧高度数据
	int height = 0;
	// 最近有效值
	USHORT depth_min_reliable_distance = 0;
	// 最远有效值
	USHORT depth_max_reliable_distance = 0;
	// 帧缓存大小
	UINT nBufferSize = 0;

	// 深度缓存
	UINT16 *pBuffer = nullptr;

	// 获取深度帧
	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);


	// 获取帧描述
	if (SUCCEEDED(hr)) {
		hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
	}
	// 获取帧宽度
	if (SUCCEEDED(hr)) {
		hr = pFrameDescription->get_Width(&width);
	}
	// 获取帧高度
	if (SUCCEEDED(hr)) {
		hr = pFrameDescription->get_Height(&height);
	}
	// 获取最近有效距离值
	if (SUCCEEDED(hr)) {
		hr = pDepthFrame->get_DepthMinReliableDistance(&depth_min_reliable_distance);
	}
	// 获取最远有效距离值
	if (SUCCEEDED(hr)) {
		hr = pDepthFrame->get_DepthMaxReliableDistance(&depth_max_reliable_distance);
	}
	// 获取深度数据
	if (SUCCEEDED(hr)) {
		hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		qDebug() << "buffer size is:" << nBufferSize;


	}
	// ----------- Fusion 处理开始
	HRESULT hr4f = hr;

	// 由原深度数据构造浮点数据
	if (SUCCEEDED(hr4f)) {
		hr4f = m_pReconstruction->DepthToDepthFloatFrame(
			pBuffer,
			nBufferSize * sizeof(UINT16),
			m_pDepthFloatImage,
			m_fMinDepthThreshold,
			m_fMaxDepthThreshold,
			true
		);

	}
	// 平滑数据
	if (SUCCEEDED(hr4f)) {
		hr4f = m_pReconstruction->SmoothDepthFloatFrame(
			m_pDepthFloatImage,
			m_pSmoothDepthFloatImage,
			1,
			0.03f
		);

	}
	// 处理当前帧
	if (SUCCEEDED(hr4f)) {
		hr4f = m_pReconstruction->ProcessFrame(
			m_pSmoothDepthFloatImage,
			/*
			*参数为配准过程中的迭代次数，
			*该参数用来表示对齐相机追踪算法的迭代次数，
			*最小值为1，值越小的计算速度更快，
			*但是设置过小会导致配准过程不收敛，从而得不到正确的转换。*/
			NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
			/*
			*参数用来控制深度影像融合的平滑参数，
			*值过小会使得的图像具有更多的噪点，
			*但是物体的移动显示的更快，消失的也更快，因此比较适合动态场景建模。
			*大的值使得物体融合的更慢，但是会保有更多的细节，噪点更少。
			*/
			m_cMaxIntegrationWeight,
			nullptr,

			&m_worldToCameraTransform
		);

	}
	// 检查错误
	if (hr4f == E_NUI_FUSION_TRACKING_ERROR) {
		qDebug()<< "Fusion tracking fail";
	}
	else if (SUCCEEDED(hr)) {
		qDebug() << "Fusion ture";
	}
	else {
		qDebug() << "Fusion tracking fail";
	}
	// 获取当前矩阵
	if (SUCCEEDED(hr4f)) {
		Matrix4 calculatedCameraPose;
		hr4f = m_pReconstruction->GetCurrentWorldToCameraTransform(&calculatedCameraPose);
		if (SUCCEEDED(hr4f)) {
			m_worldToCameraTransform = calculatedCameraPose;
		}
	}
	// 计算点云
	if (SUCCEEDED(hr4f)) {
		hr4f = m_pReconstruction->CalculatePointCloud(
			m_pPointCloud,
			&m_worldToCameraTransform
		);

	}
	// 生成图像帧
	if (SUCCEEDED(hr4f)) {
		// Shading Point Clouid
		Matrix4 worldToBGRTransform = { 0.0f };
		worldToBGRTransform.M11 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountX;
		worldToBGRTransform.M22 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountY;
		worldToBGRTransform.M33 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountZ;
		worldToBGRTransform.M41 = 0.5f;
		worldToBGRTransform.M42 = 0.5f;
		worldToBGRTransform.M43 = 0.0f;
		worldToBGRTransform.M44 = 1.0f;

		//SetIdentityMatrix(worldToBGRTransform);
		//
		hr = NuiFusionShadePointCloud(
			m_pPointCloud,
			&m_worldToCameraTransform,
			&worldToBGRTransform,
			m_pSurfaceImageFrame,
			m_pNormalImageFrame
		);

	}
	
	if (SUCCEEDED(hr4f))
	{
		RGBQUAD *pBuffer = NULL;
		pBuffer = m_pColorRGBX;
		pBuffer=reinterpret_cast<RGBQUAD*>(m_pSurfaceImageFrame->pFrameBuffer->pBits);
		//pBuffer = reinterpret_cast<RGBQUAD*>(m_pNormalImageFrame->pFrameBuffer->pBits);
		ui->label->setPixmap(QPixmap::fromImage(QImage(reinterpret_cast<uchar*>(pBuffer), width, height, QImage::Format_RGB32)).scaledToWidth(800));
	}
	//SafeRelease(pBuffer);
	SafeRelease(pFrameDescription);
	SafeRelease(pDepthFrame);
	


}


