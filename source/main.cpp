#ifdef _WIN32
#  include <winsock2.h> // make sure windows.h doesn't load winsock.h
#  include <windows.h>
#  include "Shlwapi.h"
#  pragma comment(lib, "shlwapi.lib")
#  undef max
#else
#  include <sys/time.h>
#endif

#include <algorithm>
#include <math.h>
#include <limits>
#include <iostream>
#include <cassert>
#include <string>
#include <limits>
#include <map>
#ifndef WIN32
#  include <unistd.h>
#  include <sys/wait.h>
#endif
#include <sys/types.h>
#include <cstring>
#include <stdlib.h>
#include <signal.h>
#include <stdexcept>

// Simulator SDK
#include <qi/os.hpp>
#include <alnaosim/alnaosim.h>
#include <alnaosim/alnaosim_camera_definitions.h>
#include <alrobotmodel/alrobotmodel.h>

// NaoQI SDK
#ifndef __APPLE__
#  include <alproxies/altexttospeechproxy.h>
#  include <alproxies/almemoryproxy.h>
#  include <alproxies/aldialogproxy.h>
#  include <alproxies/almotionproxy.h>
#  include <alcommon/albroker.h>
#  include <alcommon/albrokermanager.h>
#  include <alcommon/almodule.h>
#endif

// V-Rep
#include <extApi.h>

// Image writing library (used to send camera images to the browser)
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// our own code
#include "chttpd.h"
#include "threads.h"
#include "res_naosvg.h"
#include "os.h"
#include "vrep.h"
#ifndef __APPLE__
#  include "vnaobridgemodule.h"
#endif


// will be set to 1 whenever a signal occured
volatile sig_atomic_t signalOccured = 0;

void signal_handler(int) {
	signalOccured = 1;
}


std::string vrepName(std::string name) {
	std::replace(name.begin(), name.end(), '/', '_');
	return name;
}


struct Image {
	simxInt                width, height;
	std::vector<simxUChar> pixels;
};


class VNaoBridge {
public:
	Mutex globalMutex; // lock this before accessing or modifying any data

	int                            vrepPort;                // port of the V-Rep remoteAPI connection
	simxInt                        vrepClientId;            // V-Rep remoteAPI connection handle
	simxInt                        vrepNaoObjectHandle;     // V-Rep handle of the NAO robot object
	std::string                    vrepNaoObjectName;       // V-Rep name of the NAO robot object
	std::map<std::string, simxInt> vrepNaoAllObjectHandles; // index of all V-Rep objects in the subtree below the NAO robot (for quick access)

	int                 naoqiId;          // Simulator-Sdk Robot Id
	int                 naoqiHalDelay;    // Delay the start of 'hal' by this amount of seconds
	int                 naoqiBinDelay;    // Delay the start of 'naoqi-bin' by this amount of seconds
	int                 naoqiPidHal;      // ID of the running 'hal' process, or 0
	int                 naoqiPidNaoqibin; // ID of the running 'naoqi-bin' process, or 0
	std::string         naoqiPath;        // path to the simulator-sdk folder
	std::string         naoqiModelPath;   // path to the Simulator SDK xml robot description
	Sim::Model *        naoqiModel;       // NaoQI Hal Model that contains structural information about the robot
	Sim::HALInterface * naoqiHal;         // NAOQi Hal interface that allows sending and receiving data

	#ifndef __APPLE__
		boost::shared_ptr<AL::ALBroker>            naoqiBroker;
		boost::shared_ptr<AL::ALMotionProxy>       naoqiAlMotion;
		boost::shared_ptr<AL::ALTextToSpeechProxy> naoqiAlTextToSpeech;
		// boost::shared_ptr<AL::ALDialogProxy>       naoqiAlDialog;
	#endif

	u_short webserverPort;
	std::map<std::string, Image> cameraImages;
	int colorR, colorG, colorB;

	bool tactile_front;
	bool tactile_middle;
	bool tactile_rear;

	VNaoBridge() :
		vrepPort(20000),
		vrepClientId(-1),
		vrepNaoObjectHandle(0),
		vrepNaoObjectName("NAO"),
		naoqiId(9559),
		naoqiHalDelay(3),
		naoqiBinDelay(6),
		naoqiPidHal(0),
		naoqiPidNaoqibin(0),
		naoqiModel(NULL),
		naoqiHal(NULL),
		webserverPort(19559),
		colorR(90),
		colorG(90),
		colorB(90),
		tactile_front(false),
		tactile_middle(false),
		tactile_rear(false)
	{
	}

	void startHalProcess() {
		naoqiPidHal = qi::os::spawnlp((naoqiPath + "/bin/hal").c_str(), "hal",
			"-s", ("hal-ipc" + intToString(naoqiId)).c_str(),
			"-p", "HAL/Robot/Type:string=Nao",
			"-p", "HAL/Simulation:int=1",
			"-p", "HAL/Time:int=0",
			"-p", "HAL/CycleTime:int=0",
			"-p", "DCM/Time:int=0",
			"-p", "DCM/CycleTime:int=0",
			"-p", ("HAL/SimShmId:int=" + intToString(naoqiId)).c_str(),
			"-p", "HAL/Ack:int=0",
			"-p", "HAL/Nack:int=0",
			"-p", "HAL/Error:int=0",
			(char*)NULL);
	}

	void startNaoqiProcess() {
		naoqiPidNaoqibin = qi::os::spawnlp((naoqiPath + "/bin/naoqi-bin").c_str(), "naoqi-bin",
			"-p", intToString(naoqiId).c_str(),
			"--writable-path", "/tmp/SimLauncherj29Z24Y",
			(char*)NULL);
	}

	int run() {
		// launch the http server
		std::cout << "Launching web server on port " << webserverPort << " ..." << std::endl;
		launchThread(&VNaoBridge::runHttpServerThread, (void*)this);

		// load the simulator-sdk robot model
		std::cout << "Loading Simulator SDK robot model from: " << naoqiModelPath << " ..." << std::endl;
		try {
			naoqiModel = new Sim::Model(naoqiModelPath);
		}
		catch(std::exception & e) {
			std::cerr << e.what() << std::endl;
			return 1;
		}

		// connect to V-Rep
		std::cout << "Connecting to V-Rep on port " << vrepPort << " ..." << std::endl;
		vrepClientId = simxStart("127.0.0.1", vrepPort, true, true, 5000, 5);
		if(vrepClientId == -1) {
			std::cerr << "could not connect to V-Rep" << std::endl;
			return -1;
		}

		// find robot handle and name (if not already specified)
		if(vrepNaoObjectHandle == 0) {
			std::cout << "Searching for V-Rep object called " << vrepNaoObjectName << " ..." << std::endl;
			if(simxGetObjectHandle(vrepClientId, vrepNaoObjectName.c_str(), &vrepNaoObjectHandle, simx_opmode_blocking) != simx_return_ok) {
				std::cerr << "Error: could not find a robot called 'NAO'. Please specify --vrep-nao-handle" << std::endl;
				return -1;
			}
		}

		// Query V-Rep NAO object name
		std::cout << "Querying V-Rep name for object handle " << vrepNaoObjectHandle << " " << vrepNaoObjectName << " ..." << std::endl;
		vrepNaoObjectName = vrepGetObjectName(vrepClientId, vrepNaoObjectHandle);

		// query V-Rep upfront for all object handles
		std::cout << "Fetching all V-Rep object handles ..." << std::endl;
		vrepNaoAllObjectHandles = vrepGetDescendantHandles(vrepClientId, vrepNaoObjectHandle);

		std::cout << "Requesting V-Rep to stream the following sensor data ..." << std::endl;
		// initiate streaming of important V-Rep sensors
		{
			auto cameraSensors = naoqiModel->cameraSensors();
			for(auto iter = cameraSensors.begin(); iter != cameraSensors.end(); iter++) {
				const Sim::CameraSensor* cameraSensor = *iter;
				auto cameraSensorHandle = vrepNaoAllObjectHandles.find(cameraSensor->name());
				if(cameraSensorHandle != vrepNaoAllObjectHandles.end()) {
					std::cout << "- Vision Sensor " << cameraSensor->name() << std::endl;
					simxInt    vRepImageResolution[2];
					simxUChar* vRepImage;
					simxGetVisionSensorImage(vrepClientId, cameraSensorHandle->second, vRepImageResolution, &vRepImage, 0, simx_opmode_streaming);
				}
			}
		}
		{
			auto proximitySensorHandle = vrepNaoAllObjectHandles.find("Proximity_sensor");
			if(proximitySensorHandle != vrepNaoAllObjectHandles.end()) {
				std::cout << "- Proximity Sensor " << "Proximity_sensor" << std::endl;
				simxUChar detectionState;
				simxFloat detectedPoint[3];
				simxReadProximitySensor(vrepClientId, proximitySensorHandle->second, &detectionState, detectedPoint, NULL, NULL, simx_opmode_streaming);
			}
		}
		{
			auto bodyHandle = vrepNaoAllObjectHandles.find("imported_part_20_sub0");
			if(bodyHandle != vrepNaoAllObjectHandles.end()) {
				std::cout << "- Object orientation (Gyro) " << "imported_part_20_sub0" << std::endl;
				simxFloat angles[3];
				simxGetObjectOrientation(vrepClientId, bodyHandle->second, -1, angles, simx_opmode_streaming);
			}
		}
		{
			auto fsrSensors = naoqiModel->fsrSensors();
			for(auto fsrSensor = fsrSensors.begin(); fsrSensor != fsrSensors.end(); fsrSensor++) {
				std::cout << "- Force sensor " << vrepName((*fsrSensor)->name()) << std::endl;
				auto sensorHandle = vrepNaoAllObjectHandles.find(vrepName((*fsrSensor)->name()));
				if(sensorHandle != vrepNaoAllObjectHandles.end())
					simxReadForceSensor(vrepClientId, sensorHandle->second, nullptr, nullptr, nullptr, simx_opmode_streaming);
			}
		}
		{
			auto simAngleActuators = naoqiModel->angleActuators();
			for(auto angleActuator = simAngleActuators.begin(); angleActuator != simAngleActuators.end(); angleActuator++) {
				auto jointHandle = vrepNaoAllObjectHandles.find((*angleActuator)->name());
				if(jointHandle != vrepNaoAllObjectHandles.end()) {
					std::cout << "- Joint Position " << (*angleActuator)->name() << std::endl;
					simxFloat actuatorPosition;
					simxGetJointPosition(vrepClientId, jointHandle->second, &actuatorPosition, simx_opmode_streaming);
				}
			}
		}

		// initialize a virtual NAO
		std::cout << "Initializing HAL interface on port " << naoqiId << std::endl;
		naoqiHal = new Sim::HALInterface(naoqiModel, naoqiId);

		// set all angle actuators to their initial state as specified by V-Rep
		std::cout << "Propagating the initial state of all joints from V-Rep to NaoQI" << std::endl;
		{
			auto simAngleSensors = naoqiModel->angleSensors();
			for(auto simAngleSensor = simAngleSensors.begin(); simAngleSensor != simAngleSensors.end(); simAngleSensor++) {
				auto vrepJointHandle = vrepNaoAllObjectHandles.find((*simAngleSensor)->name());
				if(vrepJointHandle == vrepNaoAllObjectHandles.end())
					continue;

				simxFloat vrepPosition;
				simxGetJointPosition(vrepClientId, vrepJointHandle->second, &vrepPosition, simx_opmode_blocking);

				std::cout << "- " << (*simAngleSensor)->name() << " = " << vrepPosition << std::endl;
				naoqiHal->sendAngleSensorValue(*simAngleSensor, vrepPosition);
			}
		}

		// keep track of the current time so we can execute other processes after a specified duration
		qi::os::timeval startTime;
		if(qi::os::gettimeofday(&startTime) != 0) {
			std::cerr << "Could not read current time. Unfortunately, we need the current time to decide when to start the NAO Simulator SDK." << std::endl;
			return -1;
		}

		bool alModuleStarted = false;

		onText("Starting Main Loop...");

		// execute the main loop
		std::cout << "Starting main loop" << std::endl;
		while(signalOccured == 0 && simxGetConnectionId(vrepClientId) != -1) {
			globalMutex.lock();

			qi::os::timeval currentTime;
			if(qi::os::gettimeofday(&currentTime) != 0) {
				std::cerr << "Could not read current time. Unfortunately, we need the current time to decide when to start the NAO Simulator SDK." << std::endl;
				return -1;
			}
			qi::os::timeval relativeTime = currentTime - startTime;

			// start hal
			if(naoqiPidHal == 0 && relativeTime.tv_sec >= naoqiHalDelay) {
				onText("Starting HAL Process...");
				std::cout << "Starting HAL Process" << std::endl;
				startHalProcess();
			}

			// start naoqi-bin
			if(naoqiPidNaoqibin == 0 && relativeTime.tv_sec >= naoqiBinDelay) {
				onText("Starting NaoQI Process...");
				std::cout << "Starting NaoQI Process" << std::endl;
				startNaoqiProcess();
			}

			if(!alModuleStarted && relativeTime.tv_sec >= naoqiBinDelay + 5) {
				alModuleStarted = true;

				#ifndef __APPLE__
					std::cout << "Creating ALBroker" << std::endl;
					try {
						naoqiBroker = AL::ALBroker::createBroker("VNaoBridgeBroker", "0.0.0.0", 0, "127.0.0.1", naoqiId, 0);

						// Deal with ALBrokerManager singleton
						AL::ALBrokerManager::setInstance(naoqiBroker->fBrokerManager.lock());
						AL::ALBrokerManager::getInstance()->addBroker(naoqiBroker);
					}
					catch(const AL::ALError& /* e */) {
						std::cerr << "Faild to connect broker to 127.0.0.1:" << naoqiId << std::endl;
						AL::ALBrokerManager::getInstance()->killAllBroker();
						AL::ALBrokerManager::kill();
					}

					std::cout << "Starting ALVNaoBridgeModule" << std::endl;
					AL::ALModule::createModule<ALVNaoBridgeModule>(naoqiBroker, this);

					std::cout << "Starting ALMotion Proxy" << std::endl;
					naoqiAlMotion = naoqiBroker->getSpecialisedProxy<AL::ALMotionProxy>("ALMotion");

					std::cout << "Starting ALTextToSpeech Proxy" << std::endl;
					naoqiAlTextToSpeech = naoqiBroker->getSpecialisedProxy<AL::ALTextToSpeechProxy>("ALTextToSpeech");

				//  std::cout << "Starting ALDialog Proxy" << std::endl;
				//  naoqiAlDialog = naoqiBroker->getSpecialisedProxy<AL::ALDialogProxy>("ALDialog");
				#endif

				onText("I'm ready!");
			}

			// Angle actuators: SimSDK -> V-Rep
			{
				auto simAngleActuators = naoqiModel->angleActuators();
				for(auto simAngleActuator = simAngleActuators.begin(); simAngleActuator != simAngleActuators.end(); simAngleActuator++) {
					float simPosition = naoqiHal->fetchAngleActuatorValue(*simAngleActuator);
					if(simPosition != simPosition) // NAN -> HAL is not started yet
						continue;

					auto vrepJointHandle = vrepNaoAllObjectHandles.find((*simAngleActuator)->name());
					if(vrepJointHandle == vrepNaoAllObjectHandles.end())
						continue;

					simxFloat vrepJointPosition;
					simxGetJointPosition(vrepClientId, vrepJointHandle->second, &vrepJointPosition, simx_opmode_buffer);
					if(vrepJointPosition != simPosition)
						simxSetJointTargetPosition(vrepClientId, vrepJointHandle->second, simPosition, simx_opmode_oneshot);

					const Sim::AngleSensor* simAngleSensor = naoqiModel->angleSensor((*simAngleActuator)->name());
					naoqiHal->sendAngleSensorValue(simAngleSensor, simPosition);
				}
			}

			// Coupled sensors: SimSDK -> SimSDK
			{
				auto simCoupledActuators = naoqiModel->coupledActuators();
				for(auto simCoupledActuator = simCoupledActuators.begin(); simCoupledActuator != simCoupledActuators.end(); simCoupledActuator++) {
					float simPosition = naoqiHal->fetchCoupledActuatorValue(*simCoupledActuator);
					if(simPosition != simPosition) // NAN -> HAL is not started yet
						continue;

					const Sim::CoupledSensor* simCoupledSensor = naoqiModel->coupledSensor((*simCoupledActuator)->name());
					naoqiHal->sendCoupledSensorValue(simCoupledSensor, simPosition);
				}
			}

			// Camera sensors: V-Rep -> SimSDK
			{
				auto simCameraSensors = naoqiModel->cameraSensors();
				for(auto iter = simCameraSensors.begin(); iter != simCameraSensors.end(); iter++) {
					const Sim::CameraSensor* simCameraSensor = *iter;

					auto cameraSensorHandle = vrepNaoAllObjectHandles.find(simCameraSensor->name());
					if(cameraSensorHandle == vrepNaoAllObjectHandles.end())
						continue;

					// fetch camera sensor image from V-Rep
					simxInt    vrepImageResolution[2];
					simxUChar* vrepImage;
					auto result = simxGetVisionSensorImage(vrepClientId, cameraSensorHandle->second, vrepImageResolution, &vrepImage, 0, simx_opmode_buffer);
					if(result == simx_return_novalue_flag)
						continue;
					if(result != simx_return_ok) {
						std::cerr << "Could not get V-Rep camera image for sensor '" << simCameraSensor->name() << "', Error: " << result << std::endl;
						vrepNaoAllObjectHandles.erase(simCameraSensor->name());
						continue;
					}

					// verify that the sensor has the requested resolution
					simxInt vrepImageWidth  = vrepImageResolution[0];
					simxInt vrepImageHeight = vrepImageResolution[1];

					// flip image vertically
					{
						size_t   stride = vrepImageWidth * 3;
						uint8_t* buffer = new uint8_t[stride];
						for(size_t y = 0; y < static_cast<size_t>(vrepImageHeight / 2); y++) {
							simxUChar* line1 = vrepImage + stride * y;
							simxUChar* line2 = vrepImage + stride * (vrepImageHeight - y - 1);
							memcpy(buffer, line1, stride);
							memcpy(line1,  line2, stride);
							memcpy(line2,  buffer, stride);
						}
						delete[] buffer;
					}

					Image image;
					image.width  = vrepImageWidth;
					image.height = vrepImageHeight;
					image.pixels.assign(vrepImage, vrepImage + vrepImageWidth * 3 * vrepImageHeight);
					cameraImages[simCameraSensor->name()] = image;

					// make sure the V-Rep camera image has the right size
					{
						int simImageBufferSize, simImageWidth, simImageHeight;
						naoqiHal->cameraBufferSize(simCameraSensor, &simImageBufferSize, &simImageWidth, &simImageHeight);
						if(simImageWidth == 0 && simImageHeight == 0) // NAO requests empty images for a few seconds after start
							continue;
						if(simImageWidth != vrepImageWidth && simImageHeight != vrepImageHeight) {
							std::cerr << "V-Rep sensor '" << simCameraSensor->name() << "' has wrong resolution (V-Rep " << vrepImageWidth << "x" << vrepImageHeight << ", NaoQI " << simImageWidth << "x" << simImageHeight << ")" << std::endl;
							continue;
						}
					}

					// send camera image to NAO
					naoqiHal->sendCameraSensorValue(simCameraSensor, vrepImage);
				}
			}

			// FSR sensors: V-Rep -> SimSDK
			{
				auto simFsrSensors = naoqiModel->fsrSensors();
				for(auto iter = simFsrSensors.begin(); iter != simFsrSensors.end(); iter++) {
					const Sim::FSRSensor* simFsrSensor = *iter;

					auto vrepSensorHandle = vrepNaoAllObjectHandles.find(vrepName(simFsrSensor->name()));
					if(vrepSensorHandle == vrepNaoAllObjectHandles.end())
						continue;

					simxUChar state;
					simxFloat forceVector[3];
					simxReadForceSensor(vrepClientId, vrepSensorHandle->second, &state, forceVector, nullptr, simx_opmode_buffer);
					if(state & 0x02) // force sensor is broken
						continue;
					if(!(state & 0x01)) // data is available
						continue;

					float vrepValue = forceVector[2] / 9.81f;

					naoqiHal->sendFSRSensorValue(simFsrSensor, vrepValue);
				}
			}

			// Tactile sensors: Web Frontend -> SimSDK
			{
				auto simFrontTouchSensor = naoqiModel->tactileSensor("Head/Touch/Front");
				if(simFrontTouchSensor) {
					naoqiHal->sendTactileSensorValue(simFrontTouchSensor, tactile_front ? 1.0 : 0.0);
					tactile_front = false;
				}

				auto simMiddleTouchSensor = naoqiModel->tactileSensor("Head/Touch/Middle");
				if(simMiddleTouchSensor) {
					naoqiHal->sendTactileSensorValue(simMiddleTouchSensor, tactile_middle ? 1.0 : 0.0);
					tactile_middle = false;
				}

				auto simRearTouchSensor = naoqiModel->tactileSensor("Head/Touch/Rear");
				if(simRearTouchSensor) {
					naoqiHal->sendTactileSensorValue(simRearTouchSensor, tactile_rear ? 1.0 : 0.0);
					tactile_rear = false;
				}
			}

			// Inertial sensors: V-Rep -> SimSDK
			{
				auto simInertialSensors = naoqiModel->inertialSensors();
				for(auto iter = simInertialSensors.begin(); iter != simInertialSensors.end(); iter++) {
					const Sim::InertialSensor* simInertialSensor = *iter;

					auto vrepBodyHandle = vrepNaoAllObjectHandles.find("imported_part_20_sub0");
					if(vrepBodyHandle == vrepNaoAllObjectHandles.end())
						continue;

					simxFloat angles[3];
					const simxInt ABSOLUTE_ORIENTATION = -1;
					simxGetObjectOrientation(vrepClientId, vrepBodyHandle->second, ABSOLUTE_ORIENTATION, angles, simx_opmode_buffer);

					std::vector<float> inertialValues;
					inertialValues.push_back(angles[0]); // angleX
					inertialValues.push_back(angles[1]); // angleY
					inertialValues.push_back(0.0); // accX
					inertialValues.push_back(0.0); // accY
					inertialValues.push_back(0.0); // accZ
					inertialValues.push_back(0.0); // gyrX
					inertialValues.push_back(0.0); // gyrY

					naoqiHal->sendInertialSensorValues(simInertialSensor, inertialValues);
				}
			}

			// Sonar sensors: V-Rep -> SimSDK
			{
				auto simSonarSensors = naoqiModel->sonarSensors();
				for(auto simSonarSensor = simSonarSensors.begin(); simSonarSensor != simSonarSensors.end(); simSonarSensor++) {
					auto vrepProximitySensorHandle = vrepNaoAllObjectHandles.find("Proximity_sensor");
					if(vrepProximitySensorHandle == vrepNaoAllObjectHandles.end())
						continue;

					simxUChar detectionState;
					simxFloat detectedPoint[3];
					auto result = simxReadProximitySensor(vrepClientId, vrepProximitySensorHandle->second, &detectionState, detectedPoint, NULL, NULL, simx_opmode_buffer);
					if(result == simx_return_novalue_flag)
						continue;
					if(result != simx_return_ok) {
						std::cerr << "Could not get V-Rep sensor data for 'Proximity_sensor', Error: " << result << std::endl;
						vrepNaoAllObjectHandles.erase("Proximity_sensor");
						continue;
					}

					float vrepDistance = std::numeric_limits<float>::quiet_NaN();;
					if(detectionState != 0)
						vrepDistance = detectedPoint[2];

					naoqiHal->sendSonarSensorValue(*simSonarSensor, vrepDistance);
				}
			}

			// Bumper sensors: Thin air -> SimSDK
			{
				auto simBumperSensors = naoqiModel->bumperSensors();
				for(auto simBumperSensor = simBumperSensors.begin(); simBumperSensor != simBumperSensors.end(); simBumperSensor++) {
					float value = 0.f;

					naoqiHal->sendBumperSensorValue(*simBumperSensor, value);
				}
			}

			globalMutex.unlock();

			// artificial delay
			qi::os::msleep(50);
		}

		if(naoqiPidHal > 0)
			qi::os::kill(naoqiPidHal, SIGINT);
		if(naoqiPidNaoqibin > 0)
			qi::os::kill(naoqiPidNaoqibin, SIGINT);

		if(naoqiPidHal > 0) {
			int status;
			qi::os::waitpid(naoqiPidHal, &status);
		}
		if(naoqiPidNaoqibin > 0) {
			int status;
			qi::os::waitpid(naoqiPidNaoqibin, &status);
		}

		delete naoqiHal;

		#ifdef __linux__
			unlink("/dev/shm/hal-ipc");
		#endif

		std::cout << "shutdown" << std::endl;
		return 0;
	}

	static THREAD_RET_TYPE runHttpServerThread(void* vnaoBridge);

	void onText(const std::string & contents) {
		if(!contents.empty())
			simxCallScriptFunction(vrepClientId, vrepNaoObjectName.c_str(), sim_scripttype_childscript, "say", 0, NULL, 0, NULL, 0, NULL, contents.length(), (const simxUChar*)contents.c_str(), NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
	}

	void reset() {
		simxSetModelProperty(vrepClientId, vrepNaoObjectHandle, sim_modelproperty_not_dynamic, simx_opmode_blocking);
		simxPauseCommunication(vrepClientId, 1);
		auto simAngleActuators = naoqiModel->angleActuators();
		for(auto simAngleActuator = simAngleActuators.begin(); simAngleActuator != simAngleActuators.end(); simAngleActuator++) {
			auto vrepActuatorHandle = vrepNaoAllObjectHandles.find((*simAngleActuator)->name());
			if(vrepActuatorHandle == vrepNaoAllObjectHandles.end())
				continue;

			simxSetJointPosition(vrepClientId, vrepActuatorHandle->second, (*simAngleActuator)->startValue(), simx_opmode_oneshot);
			simxSetJointTargetPosition(vrepClientId, vrepActuatorHandle->second, (*simAngleActuator)->startValue(), simx_opmode_oneshot);

			auto sensor = naoqiModel->angleSensor((*simAngleActuator)->name());
			naoqiHal->sendAngleSensorValue(sensor, (*simAngleActuator)->startValue());
		}
		simxPauseCommunication(vrepClientId, 0);
		simxSetModelProperty(vrepClientId, vrepNaoObjectHandle, 0, simx_opmode_blocking);
	}
};


#ifndef __APPLE__
	void ALVNaoBridgeModule::callback(const std::string &key, const AL::ALValue &value, const AL::ALValue &msg) {
		bridge->onText(value);
	}
#endif


void appendToStringstream(void* ss, void* data, int size) {
	*(std::stringstream*)ss << std::string((const char*)data, (size_t)size);
}


class VNaoHttpServer : public HttpServer {
private:
	VNaoBridge* vnaoBridge;

public:
	VNaoHttpServer(VNaoBridge* vnaoBridge) : HttpServer(vnaoBridge->webserverPort), vnaoBridge(vnaoBridge) {
	}

	std::string serveStatus() {
		vnaoBridge->globalMutex.lock();
		std::stringstream data;
		data << "<html>" << "\n";
		data << "<head>" << "\n";
		data << " <style type=\"text/css\">" << "\n";
		data << "   body {" << "\n";
		data << "     background: #e0e0e0;" << "\n";
		data << "     padding-top: 80px;" << "\n";
		data << "   }" << "\n";
		data << "   body, td {" << "\n";
		data << "     font-size: 14pt;" << "\n";
		data << "     font-family: Sans-Serif;" << "\n";
		data << "   }" << "\n";
		data << "   h1 {" << "\n";
		data << "     font-size: 16pt;" << "\n";
		data << "     font-weight: bold;" << "\n";
		data << "     font-family: Sans-Serif;" << "\n";
		data << "   }" << "\n";
		data << "   .page-wrapper {" << "\n";
		data << "     margin: 0 auto;" << "\n";
		data << "     max-width: 800px;" << "\n";
		data << "   }" << "\n";
		data << "   .content-wrapper {" << "\n";
		data << "     background: linear-gradient(to bottom, #ffffff, #eeeeee);" << "\n";
		data << "     border: 1px solid #cccccc;" << "\n";
		data << "     border-radius: 20px;" << "\n";
		data << "     box-shadow: 0 5px 5px rgba(0, 0, 0, 0.1);" << "\n";
		data << "     position: relative;" << "\n";
		data << "     margin-top: 20px;" << "\n";
		data << "     padding: 30px;" << "\n";
		data << "   }" << "\n";
		data << "   .logo-wrapper {" << "\n";
		data << "     position: absolute;" << "\n";
		data << "     margin-left: auto;" << "\n";
		data << "     margin-right: auto;" << "\n";
		data << "     left: 0;" << "\n";
		data << "     right: 0;" << "\n";
		data << "     width: 128px;" << "\n";
		data << "     top: -80px;" << "\n";
		data << "     text-align: center;" << "\n";
		data << "     border-radius: 100px;" << "\n";
		data << "     padding: 20px 20px;" << "\n";
		data << "     background: linear-gradient(to bottom, #ffffff, #f8f8f8);" << "\n";
		data << "     border: 1px solid #cccccc;" << "\n";
		data << "     box-shadow: 0 5px 5px rgba(0, 0, 0, 0.2);" << "\n";
		data << "     font-size: 16pt;" << "\n";
		data << "   }" << "\n";
		data << "   .logo {" << "\n";
		data << "   }" << "\n";
		data << " </style>" << "\n";
		data << " <script>" << "\n";
		data << "   function ajax(url) {" << "\n";
		data << "     let req = new XMLHttpRequest();" << "\n";
		data << "     req.open('GET', url);" << "\n";
		data << "     req.send();" << "\n";
		data << "   }" << "\n";
		data << " </script>" << "\n";
		data << "</head>" << "\n";
		data << "<body>" << "\n";
		data << " <div class=\"page-wrapper\">" << "\n";
		data << "   <div class=\"content-wrapper\">" << "\n";
		data << "     <div class=\"logo-wrapper\">" << "\n";
		data << "       " << generateNaoSVG(vnaoBridge->colorR, vnaoBridge->colorG, vnaoBridge->colorB);
		data << "       <div class=\"robot-name\">" << vnaoBridge->vrepNaoObjectName << "</div>" << "\n";
		data << "     </div>" << "\n";

		data << "NaoQI Robot: " << vnaoBridge->naoqiModel->prettyName(2) << "<br />" << "\n";
		#ifndef __APPLE__
			data << "State: " << (vnaoBridge->naoqiAlMotion && vnaoBridge->naoqiAlMotion->robotIsWakeUp() ? "Awake" : "Resting") << "<br />" << "\n";
		#endif
		data << "HAL pid: " << vnaoBridge->naoqiPidHal << "<br />" << "\n";
		data << "NaoQI-bin pid: " << vnaoBridge->naoqiPidNaoqibin << "<br />" << "\n";
		data << "   </div>" << "\n";

		data << "   <div class=\"content-wrapper\">" << "\n";
		data << "     <h1>Commands</h1>" << "\n";
		#ifndef __APPLE__
			data << "       <button onclick='ajax(\"/wake\");'>Wake Up</button>" << "\n";
			data << "       <button onclick='ajax(\"/rest\");'>Rest</button>" << "\n";
			data << "       <button onclick='ajax(\"/walk\");'>Walk 1m</button>" << "\n";
			data << "       <button onclick='ajax(\"/move0\");'>Walk 1mm</button>" << "\n";
			data << "       <button onclick='ajax(\"/stopmove\");'>Stop Moving</button>" << "\n";
		#endif
		data << "       <button onclick='ajax(\"/say\");'>Say 'Hello world'</button>" << "\n";
		// data << "        <button onclick='ajax(\"/hear\");'>Hear 'Hello world'</button>" << "\n";
		data << "       <button onclick='ajax(\"/reset\");'>Reset Joints</button>" << "\n";
		data << "       <br />" << "\n";
		data << "       <button onclick='ajax(\"/tactile_front\");'>Touch Front Head</button>" << "\n";
		data << "       <button onclick='ajax(\"/tactile_middle\");'>Touch Middle Head</button>" << "\n";
		data << "       <button onclick='ajax(\"/tactile_rear\");'>Touch Rear Head</button>" << "\n";
		data << "   </div>" << "\n";

		data << "   <div class=\"content-wrapper\">" << "\n";
		data << "     <h1>Cameras</h1>" << "\n";
		data << "     <img src=\"/CameraTop.png\" width=\"320\" height=\"240\" style=\"border: 1px solid #444;\" /> " << "\n";
		data << "     <img src=\"/CameraBottom.png\" width=\"320\" height=\"240\" style=\"border: 1px solid #444;\" /><br />" << "\n";
		data << "   </div>" << "\n";

		data << "   <div class=\"content-wrapper\">" << "\n";
		data << "     <h1>Sensors</h1>" << "\n";
		data << "     <table>" << "\n";
		{
			data << "<tr><td colspan=\"2\">Angle Actuators</td></tr>" << "\n";
			auto simAngleActuators = vnaoBridge->naoqiModel->angleActuators();
			for(auto simAngleActuator = simAngleActuators.begin(); simAngleActuator != simAngleActuators.end(); simAngleActuator++) {
				float simPosition = vnaoBridge->naoqiHal->fetchAngleActuatorValue(*simAngleActuator);
				data << "<tr><td>" << (*simAngleActuator)->name() << ":</td><td>" << simPosition << "</td></tr>" << "\n";
			}
		}
		{
			data << "<tr><td colspan=\"2\">Coupled Actuators</td></tr>" << "\n";
			auto simCoupledActuators = vnaoBridge->naoqiModel->coupledActuators();
			for(auto simCoupledActuator = simCoupledActuators.begin(); simCoupledActuator != simCoupledActuators.end(); simCoupledActuator++) {
				float simPosition = vnaoBridge->naoqiHal->fetchCoupledActuatorValue(*simCoupledActuator);
				data << "<tr><td>" << (*simCoupledActuator)->name() << ":</td><td>" << simPosition << "</td></tr>" << "\n";
			}
		}
		{
			data << "<tr><td colspan=\"2\">FSR Sensors</td></tr>" << "\n";
			auto simFsrSensors = vnaoBridge->naoqiModel->fsrSensors();
			for(auto iter = simFsrSensors.begin(); iter != simFsrSensors.end(); iter++) {
				const Sim::FSRSensor* simFsrSensor = *iter;

				auto vrepSensorHandle = vnaoBridge->vrepNaoAllObjectHandles.find(vrepName(simFsrSensor->name()));
				if(vrepSensorHandle == vnaoBridge->vrepNaoAllObjectHandles.end())
					continue;

				simxUChar state;
				simxFloat forceVector[3];
				simxReadForceSensor(vnaoBridge->vrepClientId, vrepSensorHandle->second, &state, forceVector, nullptr, simx_opmode_buffer);
				float vrepValue = forceVector[2] / 9.81f;
				data << "<tr><td>" << simFsrSensor->name() << ":</td><td>" << vrepValue << " (status: " << (int)state << ")</td></tr>" << "\n";
			}
		}
		{
			data << "<tr><td colspan=\"2\">Inertial Sensors</td></tr>" << "\n";
			auto simInertialSensors = vnaoBridge->naoqiModel->inertialSensors();
			for(auto iter = simInertialSensors.begin(); iter != simInertialSensors.end(); iter++) {
				const Sim::InertialSensor* simInertialSensor = *iter;

				auto vrepBodyHandle = vnaoBridge->vrepNaoAllObjectHandles.find("imported_part_20_sub0");
				if(vrepBodyHandle == vnaoBridge->vrepNaoAllObjectHandles.end())
					continue;

				simxFloat angles[3];
				const simxInt ABSOLUTE_ORIENTATION = -1;
				simxGetObjectOrientation(vnaoBridge->vrepClientId, vrepBodyHandle->second, ABSOLUTE_ORIENTATION, angles, simx_opmode_buffer);

				std::vector<float> inertialValues;
				inertialValues.push_back(angles[0]); // angleX
				inertialValues.push_back(angles[1]); // angleY
				inertialValues.push_back(0.0); // accX
				inertialValues.push_back(0.0); // accY
				inertialValues.push_back(0.0); // accZ
				inertialValues.push_back(0.0); // gyrX
				inertialValues.push_back(0.0); // gyrY

				data << "<tr><td>" << simInertialSensor->name() << ":</td><td>x=" << angles[0] << ", y=" << angles[1] << "</td></tr>" << "\n";
			}
		}
		{
			data << "<tr><td colspan=\"2\">Promixity Sensors</td></tr>" << "\n";
			auto simSonarSensors = vnaoBridge->naoqiModel->sonarSensors();
			for(auto simSonarSensor = simSonarSensors.begin(); simSonarSensor != simSonarSensors.end(); simSonarSensor++) {
				auto vrepProximitySensorHandle = vnaoBridge->vrepNaoAllObjectHandles.find("Proximity_sensor");
				if(vrepProximitySensorHandle == vnaoBridge->vrepNaoAllObjectHandles.end())
					continue;

				simxUChar detectionState;
				simxFloat detectedPoint[3];
				auto result = simxReadProximitySensor(vnaoBridge->vrepClientId, vrepProximitySensorHandle->second, &detectionState, detectedPoint, NULL, NULL, simx_opmode_buffer);
				if(result == simx_return_novalue_flag)
					continue;
				if(result != simx_return_ok)
					continue;

				float vrepDistance = std::numeric_limits<float>::quiet_NaN();;
				if(detectionState != 0)
					vrepDistance = detectedPoint[2];

				data << "<tr><td>" << (*simSonarSensor)->name() << ":</td><td>" << vrepDistance << "</td></tr>" << "\n";
			}
		}
		{
			data << "<tr><td colspan=\"2\">Bumper Sensors</td></tr>" << "\n";
			auto simBumperSensors = vnaoBridge->naoqiModel->bumperSensors();
			for(auto simBumperSensor = simBumperSensors.begin(); simBumperSensor != simBumperSensors.end(); simBumperSensor++) {
				float value = 0.f;
				data << "<tr><td>" << (*simBumperSensor)->name() << ":</td><td>" << value << "</td></tr>" << "\n";
			}
		}
		data << "     </table>" << "\n";
		data << "   </div>" << "\n";

		data << " </div>" << "\n";
		data << "</body>" << "\n";
		data << "</html>" << "\n";
		vnaoBridge->globalMutex.unlock();
		return HttpServer::generate200Response(data.str(), "text/html; charset=utf-8");
	}

	std::string handleGet(const std::string & requestPath) {
		if(requestPath == "/")
			return serveStatus();
		else if(requestPath == "/reset") {
			vnaoBridge->globalMutex.lock();
			vnaoBridge->reset();
			vnaoBridge->globalMutex.unlock();
			return HttpServer::generate200Response("ok", "text/html; charset=utf-8");
		}
		#ifndef __APPLE__
			else if(requestPath == "/say") {
				vnaoBridge->globalMutex.lock();
				if(vnaoBridge->naoqiAlTextToSpeech)
					async(boost::bind(&AL::ALTextToSpeechProxy::say, vnaoBridge->naoqiAlTextToSpeech, "Hello world"));
				vnaoBridge->globalMutex.unlock();
				return HttpServer::generate200Response("ok", "text/html; charset=utf-8");
			}
			else if(requestPath == "/walk") {
				vnaoBridge->globalMutex.lock();
				if(vnaoBridge->naoqiAlMotion)
					async(boost::bind(&AL::ALMotionProxy::moveTo, vnaoBridge->naoqiAlMotion, 1.0, 0.0, 0.0));
				vnaoBridge->globalMutex.unlock();
				return HttpServer::generate200Response("ok", "text/html; charset=utf-8");
			}
			else if(requestPath == "/wake") {
				vnaoBridge->globalMutex.lock();
				if(vnaoBridge->naoqiAlMotion)
					async(boost::bind(&AL::ALMotionProxy::wakeUp, vnaoBridge->naoqiAlMotion));
				vnaoBridge->globalMutex.unlock();
				return HttpServer::generate200Response("ok", "text/html; charset=utf-8");
			}
			else if(requestPath == "/stopmove") {
				vnaoBridge->globalMutex.lock();
				if(vnaoBridge->naoqiAlMotion)
					async(boost::bind(&AL::ALMotionProxy::stopMove, vnaoBridge->naoqiAlMotion));
				vnaoBridge->globalMutex.unlock();
				return HttpServer::generate200Response("ok", "text/html; charset=utf-8");
			}
			else if(requestPath == "/move0") {
				vnaoBridge->globalMutex.lock();
				if(vnaoBridge->naoqiAlMotion)
					async(boost::bind(&AL::ALMotionProxy::moveTo, vnaoBridge->naoqiAlMotion, 0.001, 0.0, 0.0));
				vnaoBridge->globalMutex.unlock();
				return HttpServer::generate200Response("ok", "text/html; charset=utf-8");
			}
			else if(requestPath == "/rest") {
				vnaoBridge->globalMutex.lock();
				if(vnaoBridge->naoqiAlMotion)
					async(boost::bind(&AL::ALMotionProxy::rest, vnaoBridge->naoqiAlMotion));
				vnaoBridge->globalMutex.unlock();
				return HttpServer::generate200Response("ok", "text/html; charset=utf-8");
			}
		#endif
		else if(requestPath == "/tactile_front") {
			vnaoBridge->tactile_front = true;
			return HttpServer::generate200Response("ok", "text/html; charset=utf-8");
		}
		else if(requestPath == "/tactile_middle") {
			vnaoBridge->tactile_middle = true;
			return HttpServer::generate200Response("ok", "text/html; charset=utf-8");
		}
		else if(requestPath == "/tactile_rear") {
			vnaoBridge->tactile_rear = true;
			return HttpServer::generate200Response("ok", "text/html; charset=utf-8");
		}
		// else if(requestPath == "/hear") {
		//  vnaoBridge->globalMutex.lock();
		//  if(vnaoBridge->naoqiAlDialog)
		//    async(boost::bind(&AL::ALDialogProxy::forceInput, vnaoBridge->naoqiAlDialog, "Hello world"));
		//  vnaoBridge->globalMutex.unlock();
		//  return HttpServer::generate200Response("ok", "text/html; charset=utf-8");
		// }
		else if(requestPath == "/CameraTop.png" || requestPath == "/CameraBottom.png") {
			vnaoBridge->globalMutex.lock();

			Image image;
			image.width  = 1;
			image.height = 1;
			image.pixels.push_back(0);
			image.pixels.push_back(0);
			image.pixels.push_back(0);

			auto iter = vnaoBridge->cameraImages.find((requestPath == "/CameraTop.png") ? "CameraTop" : "CameraBottom");
			if(iter != vnaoBridge->cameraImages.end())
				image = iter->second;

			std::stringstream data;
			stbi_write_png_to_func(&appendToStringstream, &data, image.width, image.height, 3, image.pixels.data(), image.width * 3);

			vnaoBridge->globalMutex.unlock();
			return HttpServer::generate200Response(data.str(), "image/png");
		}
		else
			return HttpServer::generate404Response();
	}
};


THREAD_RET_TYPE VNaoBridge::runHttpServerThread(void* vnaoBridge) {
	VNaoHttpServer server((VNaoBridge*)vnaoBridge);
	server.run();

	#if defined (__linux) || defined (__APPLE__)
		return NULL;
	#endif
}


int main(int argc, char* argv[]) {
	VNaoBridge vNaoBridge;

	// locate the model and the simulator-sdk (we expect vnaobridge to be located in .../V-Rep.../simulator-sdk/bin/vnaobridge)
	vNaoBridge.naoqiPath      = getCurrentExecutableDirectory(argv[0]) + "/..";
	vNaoBridge.naoqiModelPath = vNaoBridge.naoqiPath + "/share/alrobotmodel/models/NAOH25V50.xml";

	// handle command line arguments
	for(int i = 1; i < argc; ) {
		if(std::string(argv[i]) == "--help") {
			std::cout << "Usage: vnaobridge [options]" << std::endl;
			std::cout << "Options:" << std::endl;
			std::cout << "  --naoqi-id 9559      ID (and Port) used to run the Simulator-SDK" << std::endl;
			std::cout << "  --naoqi-hal-delay 3  Delay the start of hal by n seconds" << std::endl;
			std::cout << "  --naoqi-bin-delay 6  Delay the start of naoqi-bin by n seconds" << std::endl;
			std::cout << "  --vrep-port 20000    Port used to connect to V-Rep" << std::endl;
			std::cout << "  --vrep-nao-handle 0  V-Rep handle to the NAO robot" << std::endl;
			std::cout << "                       If set to 0 we search for an object with the" << std::endl;
			std::cout << "                       name specified in --vrep-nao-name." << std::endl;
			std::cout << "  --vrep-nao-name NAO  V-Rep name to search for if --vrep-nao-handle" << std::endl;
			std::cout << "                       is not specified" << std::endl;
			std::cout << "  --color r g b        Robot accent color (0-255)" << std::endl;
			return 0;
		}
		else if(std::string(argv[i]) == "--naoqi-id") {
			if(i + 1 >= argc) {
				std::cerr << "error: missing argument to --naoqi-id" << std::endl;
				return 1;
			}
			if(intToString(atoi(argv[i + 1])) != argv[i + 1]) {
				std::cerr << "error: bad argument to --naoqi-id: " << argv[i + 1] << std::endl;
				return 1;
			}
			vNaoBridge.naoqiId = atoi(argv[i + 1]);
			vNaoBridge.webserverPort = 10000 + vNaoBridge.naoqiId;
			i += 2;
		}
		else if(std::string(argv[i]) == "--naoqi-hal-delay") {
			if(i + 1 >= argc) {
				std::cerr << "error: missing argument to --naoqi-hal-delay" << std::endl;
				return 1;
			}
			if(intToString(atoi(argv[i + 1])) != argv[i + 1]) {
				std::cerr << "error: bad argument to --naoqi-hal-delay: " << argv[i + 1] << std::endl;
				return 1;
			}
			vNaoBridge.naoqiHalDelay = atoi(argv[i + 1]);
			i += 2;
		}
		else if(std::string(argv[i]) == "--naoqi-bin-delay") {
			if(i + 1 >= argc) {
				std::cerr << "error: missing argument to --naoqi-bin-delay" << std::endl;
				return 1;
			}
			if(intToString(atoi(argv[i + 1])) != argv[i + 1]) {
				std::cerr << "error: bad argument to --naoqi-bin-delay: " << argv[i + 1] << std::endl;
				return 1;
			}
			vNaoBridge.naoqiBinDelay = atoi(argv[i + 1]);
			i += 2;
		}
		else if(std::string(argv[i]) == "--vrep-port") {
			if(i + 1 >= argc) {
				std::cerr << "error: missing argument to --vrep-port" << std::endl;
				return 1;
			}
			if(intToString(atoi(argv[i + 1])) != argv[i + 1]) {
				std::cerr << "error: bad argument to --vrep-port: " << argv[i + 1] << std::endl;
				return 1;
			}
			vNaoBridge.vrepPort = atoi(argv[i + 1]);
			i += 2;
		}
		else if(std::string(argv[i]) == "--vrep-nao-handle") {
			if(i + 1 >= argc) {
				std::cerr << "error: missing argument to --vrep-nao-handle" << std::endl;
				return 1;
			}
			if(intToString(atoi(argv[i + 1])) != argv[i + 1]) {
				std::cerr << "error: bad argument to --vrep-nao-handle: " << argv[i + 1] << std::endl;
				return 1;
			}
			vNaoBridge.vrepNaoObjectHandle = atoi(argv[i + 1]);
			i += 2;
		}
		else if(std::string(argv[i]) == "--vrep-nao-name") {
			if(i + 1 >= argc) {
				std::cerr << "error: missing argument to --vrep-nao-name" << std::endl;
				return 1;
			}
			vNaoBridge.vrepNaoObjectName = argv[i + 1];
			i += 2;
		}
		else if(std::string(argv[i]) == "--color") {
			if(i + 3 >= argc) {
				std::cerr << "error: missing argument to --color" << std::endl;
				return 1;
			}
			if(intToString(atoi(argv[i + 1])) != argv[i + 1]) {
				std::cerr << "error: bad argument to --color: " << argv[i + 1] << std::endl;
				return 1;
			}
			if(intToString(atoi(argv[i + 2])) != argv[i + 2]) {
				std::cerr << "error: bad argument to --color: " << argv[i + 2] << std::endl;
				return 1;
			}
			if(intToString(atoi(argv[i + 3])) != argv[i + 3]) {
				std::cerr << "error: bad argument to --color: " << argv[i + 3] << std::endl;
				return 1;
			}
			vNaoBridge.colorR = atoi(argv[i + 1]);
			vNaoBridge.colorG = atoi(argv[i + 2]);
			vNaoBridge.colorB = atoi(argv[i + 3]);
			i += 4;
		}
		else {
			std::cerr << "error: unknown argument " << argv[i] << std::endl;
			return 1;
		}
	}

	// install a signal handler to gracefully terminate the process when the user requests it
	#ifndef WIN32
		struct sigaction new_action;
		new_action.sa_handler = signal_handler;
		sigemptyset(&new_action.sa_mask);
		new_action.sa_flags = 0;
		sigaction(SIGINT, &new_action, NULL);
		sigaction(SIGTERM, &new_action, NULL);
	#else
		signal(SIGINT, signal_handler);
	#endif

	// Simulator SDK on MacOS contains a lot of messed up library paths.
	// Most of those can be fixed with the script fix-naoqi.sh, but for some we have to fool the linker:
	#ifdef __APPLE__
		qi::os::setenv("DYLD_LIBRARY_PATH", (vNaoBridge.naoqiPath + "/lib/").c_str());
	#endif

	// NaoQI needs this for some SOAP-related number parsing
	setlocale(LC_NUMERIC, "C");

	return vNaoBridge.run();
}
