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
				//			�ؽ�����
				&m_reconstructionParams,
				//cpuģʽ����gpuģʽ
				m_processorType,
				//������һ���豸 Ĭ��-1
				m_deviceIndex,
				//				���絽�����ת��
				&m_worldToCameraTransform,
				//				�������
				&m_pReconstruction
			);
		}
		if (FAILED(hr)) { qDebug() << "error!"; }
		if (SUCCEEDED(hr)) {
			hr = m_pReconstruction->GetCurrentWorldToVolumeTransform(&m_defaultWorldToVolumeTransform);
		}
		// �����������֡
		if (SUCCEEDED(hr)) {
			hr = NuiFusionCreateImageFrame(
				NUI_FUSION_IMAGE_TYPE_FLOAT,
				m_cDepthWidth,
				m_cDepthHeight,
				nullptr,
				&m_pDepthFloatImage
			);
		}
		// ����ƽ���������֡
		if (SUCCEEDED(hr)) {
			hr = NuiFusionCreateImageFrame(
				NUI_FUSION_IMAGE_TYPE_FLOAT,
				m_cDepthWidth,
				m_cDepthHeight,
				nullptr,
				&m_pSmoothDepthFloatImage
			);
		}
		// ��������֡
		if (SUCCEEDED(hr)) {
			hr = NuiFusionCreateImageFrame(
				NUI_FUSION_IMAGE_TYPE_POINT_CLOUD,
				m_cDepthWidth,
				m_cDepthHeight,
				nullptr,
				&m_pPointCloud
			);
		}
		// ����Fusionͼ��֡
		if (SUCCEEDED(hr)) {
			hr = NuiFusionCreateImageFrame(
				NUI_FUSION_IMAGE_TYPE_COLOR,
				m_cDepthWidth,
				m_cDepthHeight,
				nullptr,
				&m_pSurfaceImageFrame
			);
		}
		// ����Fusion����֡
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


	// ���֡
	IDepthFrame* pDepthFrame = nullptr;
	// ֡����
	IFrameDescription* pFrameDescription = nullptr;
	// ���֡�������
	int width = 0;
	// ���֡�߶�����
	int height = 0;
	// �����Чֵ
	USHORT depth_min_reliable_distance = 0;
	// ��Զ��Чֵ
	USHORT depth_max_reliable_distance = 0;
	// ֡�����С
	UINT nBufferSize = 0;

	// ��Ȼ���
	UINT16 *pBuffer = nullptr;

	// ��ȡ���֡
	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);


	// ��ȡ֡����
	if (SUCCEEDED(hr)) {
		hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
	}
	// ��ȡ֡���
	if (SUCCEEDED(hr)) {
		hr = pFrameDescription->get_Width(&width);
	}
	// ��ȡ֡�߶�
	if (SUCCEEDED(hr)) {
		hr = pFrameDescription->get_Height(&height);
	}
	// ��ȡ�����Ч����ֵ
	if (SUCCEEDED(hr)) {
		hr = pDepthFrame->get_DepthMinReliableDistance(&depth_min_reliable_distance);
	}
	// ��ȡ��Զ��Ч����ֵ
	if (SUCCEEDED(hr)) {
		hr = pDepthFrame->get_DepthMaxReliableDistance(&depth_max_reliable_distance);
	}
	// ��ȡ�������
	if (SUCCEEDED(hr)) {
		hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		qDebug() << "buffer size is:" << nBufferSize;


	}
	// ----------- Fusion ����ʼ
	HRESULT hr4f = hr;

	// ��ԭ������ݹ��측������
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
	// ƽ������
	if (SUCCEEDED(hr4f)) {
		hr4f = m_pReconstruction->SmoothDepthFloatFrame(
			m_pDepthFloatImage,
			m_pSmoothDepthFloatImage,
			1,
			0.03f
		);

	}
	// ����ǰ֡
	if (SUCCEEDED(hr4f)) {
		hr4f = m_pReconstruction->ProcessFrame(
			m_pSmoothDepthFloatImage,
			/*
			*����Ϊ��׼�����еĵ���������
			*�ò���������ʾ�������׷���㷨�ĵ���������
			*��СֵΪ1��ֵԽС�ļ����ٶȸ��죬
			*�������ù�С�ᵼ����׼���̲��������Ӷ��ò�����ȷ��ת����*/
			NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
			/*
			*���������������Ӱ���ںϵ�ƽ��������
			*ֵ��С��ʹ�õ�ͼ����и������㣬
			*����������ƶ���ʾ�ĸ��죬��ʧ��Ҳ���죬��˱Ƚ��ʺ϶�̬������ģ��
			*���ֵʹ�������ںϵĸ��������ǻᱣ�и����ϸ�ڣ������١�
			*/
			m_cMaxIntegrationWeight,
			nullptr,

			&m_worldToCameraTransform
		);

	}
	// ������
	if (hr4f == E_NUI_FUSION_TRACKING_ERROR) {
		qDebug()<< "Fusion tracking fail";
	}
	else if (SUCCEEDED(hr)) {
		qDebug() << "Fusion ture";
	}
	else {
		qDebug() << "Fusion tracking fail";
	}
	// ��ȡ��ǰ����
	if (SUCCEEDED(hr4f)) {
		Matrix4 calculatedCameraPose;
		hr4f = m_pReconstruction->GetCurrentWorldToCameraTransform(&calculatedCameraPose);
		if (SUCCEEDED(hr4f)) {
			m_worldToCameraTransform = calculatedCameraPose;
		}
	}
	// �������
	if (SUCCEEDED(hr4f)) {
		hr4f = m_pReconstruction->CalculatePointCloud(
			m_pPointCloud,
			&m_worldToCameraTransform
		);

	}
	// ����ͼ��֡
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


