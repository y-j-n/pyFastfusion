
#ifndef ONLINEFUSIONVIEWER_IF_HPP_
#define ONLINEFUSIONVIEWER_IF_HPP_

#include <QGLViewer/qglviewer.h>
#include <camerautils/camerautils.hpp>
//#include <fusionGPU/geometryfusion_single_aos.hpp>
#include <fusion/geometryfusion_mipmap_cpu.hpp>
#include <fusion/mesh.hpp>
#include <QtCore/QTimer>

typedef struct FusionParameter_{
  FusionMipMapCPU *fusion;
  float imageDepthScale;
  float maxCamDistance;
  bool threadImageReading;
  size_t stopFrame;
  FusionParameter_(FusionMipMapCPU *fusion_p, float imageDepthScale_p, float maxCamDistance_p, bool threadImageReading_p, size_t stopFrame_p)
  :fusion(fusion_p),imageDepthScale(imageDepthScale_p),maxCamDistance(maxCamDistance_p), threadImageReading(threadImageReading_p)
  ,stopFrame(stopFrame_p){}
} FusionParameter;

typedef struct {
    //---- members required by fusionWrapperIfOneStep()
    volatile size_t currentFrame;

//    std::vector<std::string> &depthLast;
    std::vector<std::vector<std::string> > *depthNames;

//    std::vector<std::string> &rgbLast;
    std::vector<std::vector<std::string> > *rgbNames;

//    std::vector<CameraInfo> &pLast;
	std::vector<std::vector<CameraInfo> > *poses;

    FusionMipMapCPU *fusion;
    float imageDepthScale;
    float maxCamDistance;
    volatile bool *newMesh;
    //---- other members
    bool threadImageReading;
    size_t stopFrame;
    unsigned int startFrame;
    unsigned int fusedFrames;
    size_t lastFrame;
} FusionWrapperIfData;



class OnlineFusionViewerInterface : public QGLViewer
{
	Q_OBJECT
public:
	OnlineFusionViewerInterface(bool createMeshList = true);
	~OnlineFusionViewerInterface();
	void updateSlotIf();

    static FusionWrapperIfData * fusionWrapperIfInit(
	    std::vector<std::vector<std::string> > *depthNames,
	    std::vector<std::vector<std::string> > *rgbNames,
	    std::vector<std::vector<CameraInfo> > *poses,
	    FusionParameter par,
	    volatile long int *_currentFrame,
	    volatile long int *_currentTrajectory,
	    volatile bool *newMesh,
	    volatile bool *fusionActive,
	    volatile bool *fusionAlive);

    static void fusionWrapperIfOneStep(
        volatile size_t currentFrame,
        std::vector<std::string> *depthLast,
        std::vector<std::string> *rgbLast,
        FusionMipMapCPU *fusion,
        std::vector<CameraInfo> *pLast,
        float imageDepthScale,
        float maxCamDistance,
        volatile bool *newMesh
    );

	static void fusionWrapperIf(
	    FusionWrapperIfData *pFusionWrapperIfData,
//	    std::vector<std::vector<std::string> > depthNames,
//	    std::vector<std::vector<std::string> > rgbNames,
//	    std::vector<std::vector<CameraInfo> > poses,
//	    FusionParameter par,
	    volatile long int *_currentFrame,
	    volatile long int *_currentTrajectory,
//	    volatile bool *newMesh,
	    volatile bool *fusionActive,
		volatile bool *fusionAlive
	);

  std::vector<float> _boundingBox;
  std::vector<std::vector<CameraInfo> > _poses;
  std::vector<std::vector<std::string> > _depthNames;
  std::vector<std::vector<std::string> > _rgbNames;
//  std::vector<Mesh> _meshes;
  MeshSeparate *_currentMeshForSave;
  MeshInterleaved *_currentMeshInterleaved;
  std::vector<PointerMeshDraw*> _pointermeshes;
  FusionMipMapCPU* _fusion;
  float _imageDepthScale;
  float _maxCamDistance;
  long int _currentFrame;
  long int _currentTrajectory;
  long int _firstFrame;
  long int _nextStopFrame;

  bool _threadFusion;
  boost::thread *_fusionThread;
  bool _newMesh;
  bool _fusionActive;
  bool _fusionAlive;

  bool _threadImageReading;

protected slots:
void updateSlot();


protected :
  virtual void init();
  virtual void draw();
  virtual void fastDraw();
  void drawMeshPointer();
  void drawMeshInterleaved();
  void drawCameraFrustum(CameraInfo pose,std::string imageName);
  virtual void keyPressEvent(QKeyEvent *e);
  void drawGridFrame(float ox, float oy, float oz, float sx, float sy, float sz);
  void setScenePosition(CameraInfo pose);

  void enableLighting();
  void disableLighting();


  long long _lastComputedFrame;

  bool _verbose;
  bool _showCameraFrustum;
  bool _showGridBoundingbox;
  bool _showDepthImage;
  bool _showColor;
  int _displayMode;

  unsigned int _meshNumber;
  unsigned int _fusionNumber;
  float _cx; float _cy; float _cz;

  std::vector<PointerMeshDraw*> _meshesDraw;
  PointerMeshDraw *_currentMesh;
  unsigned int _currentNV;
  unsigned int _currentNF;
  int _currentMeshType;

  GLuint _vertexBuffer;
  GLuint _faceBuffer;
  GLuint _edgeBuffer;
  GLuint _normalBuffer;
  GLuint _colorBuffer;
  unsigned int _vertexBufferSize;
  unsigned int _faceBufferSize;
  unsigned int _edgeBufferSize;

  bool _onTrack;
  bool _onInterpolation;
  bool _saving;

  qglviewer::KeyFrameInterpolator _interpolator;
  std::vector<qglviewer::ManipulatedFrame*> _keyFrames;

  bool _runFusion;
  bool _createMeshList;
  bool _lightingEnabled;
  bool _colorEnabled;

  bool _lightingInternal;



  QTimer *_timer;

  void generateBuffers();
  bool _buffersGenerated;

};


#endif /* ONLINEFUSIONVIEWER_IF_HPP_ */
